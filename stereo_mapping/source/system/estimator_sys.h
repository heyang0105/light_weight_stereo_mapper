
//2021-03-01
#include<memory>
#include"util/threading.h"
namespace stereo{

namespace inner{
    struct ImageData;
}

class StereoEstimator : public Thread{
public:
    struct Options{

    };

private:
    void Run();

};

}