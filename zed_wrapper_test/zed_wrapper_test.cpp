/*******************************************************
 * Copyright (C) 2021
 * 
 * This file is part of a personal project.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: He Yang (heyang0105@126.com)
 *******************************************************/
#include <iostream>
#include <queue>
#include <map>
#include <thread>
#include <string>
#include <mutex>
#include <memory>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include"estimator_sys.h"

using namespace std;

unique_ptr<stereo_mapper::StereoEstimator> sys_estimator_ptr;

string IMAGE0_TOPIC = "/zed/left/image_raw_color";
string IMAGE1_TOPIC = "/zed/right/image_raw_color";

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

void img0Callback(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1Callback(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat img = cv_ptr->image.clone();
    return img;
}

void syncProcess(){
    while(1){
        

        bool is_new_pair_arrived = false;
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0;

        m_buf.lock();
        if (!img0_buf.empty() && !img1_buf.empty()){
            double time0 = img0_buf.front()->header.stamp.toSec();
            double time1 = img1_buf.front()->header.stamp.toSec();
            if(time0 < time1){
                img0_buf.pop();
                cout<<"throw img0\n";
            }
            else if(time0 > time1){
                img1_buf.pop();
                cout<<"throw img1\n";
            }
            else{
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image0 = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                image1 = getImageFromMsg(img1_buf.front());
                img1_buf.pop();
                is_new_pair_arrived = true;
            }
        }
        m_buf.unlock();

        if(!is_new_pair_arrived)
            continue;

        /* VIS IMAGES */
        int width0 = image0.cols;
        int width1 = image1.cols;
        int height = image0.rows;

        cv::Mat des, s0, s1;
        cv::resize(image0, s0, cv::Size2i(width0/4,  height/4));
        cv::resize(image1, s1, cv::Size2i(width1/4,  height/4));

        des.create(height/4, (width0 + width1)/4, image0.type());

        cv::Mat r0 = des(cv::Rect(0, 0, width0/4, height/4));
        s0.copyTo(r0);
        cv::Mat r1 = des(cv::Rect(width0/4, 0, width1/4, height/4));
        s1.copyTo(r1);

        cv::imshow("des", des);
        

        /* ENTRANCE OF THE MAPPING */
        sys_estimator_ptr->InsertData(time, image0, image1);

        cv::waitKey(50);
        // std::chrono::milliseconds dura(2);
        // std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber sub_img0 = n.subscribe<sensor_msgs::Image>(IMAGE0_TOPIC, 20, img0Callback);
    ros::Subscriber sub_img1 = n.subscribe<sensor_msgs::Image>(IMAGE1_TOPIC, 20, img1Callback);

    stereo_mapper::StereoEstimator::Options opt;
    sys_estimator_ptr.reset(new stereo_mapper::StereoEstimator(opt));

    std::thread sync_Thread{syncProcess};
    ros::spin();

    return 0;
}