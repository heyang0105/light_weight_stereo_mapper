// 2021-04-02
/*
    As a templat of the ransac, the estimator class must
    have a member M_t, X_t, Y_t

    key template elements:
        1. sampler
            -get rand_x, rand_y from X_t, Y_t
        2. the way to choose better model
        3. estimator
            - calc the M_t
            - provide the residuals of all X_t, Y_t if using M_t

    CORE:
        we have to set a good estimation of "max_error"
*/

#ifndef VIEW_GEO_RANSAC_RANSAC_H
#define VIEW_GEO_RANSAC_RANSAC_H

#include <cfloat>
#include <random>
#include <stdexcept>
#include <vector>
#include <assert.h>

// we use these two as the defualt tempalte arg
#include "ransac/random_sampler.h"
#include "ransac/support_measurement.h"

namespace ViewGeo{

struct RANSACOptions {

    /* Maximum error for a sample to be considered as an inlier. 
    Note that the residual of an estimator corresponds to 
    a squared error. */
    double max_error = 0.0;

    /* A priori assumed minimum inlier ratio, which determines the maximum number
     of iterations. Only applies if smaller than `max_num_trials`*/
    double min_inlier_ratio = 0.1;// PAY ATTENTION HOW IT CALC

    /* Abort the iteration if minimum probability 
    that one sample is free from outliers is reached */
    double confidence = 0.99;

    /* The num_trials_multiplier to the dynamically computed maximum number of
    iterations based on the specified confidence value.
    i.e. n times larger than the orig value */
    double dyn_num_trials_multiplier = 3.0;

    /* Number of random trials to estimate model from random subset. */
    size_t min_num_trials = 0;
    size_t max_num_trials = std::numeric_limits<size_t>::max();

    void Check() const {
        
        assert(max_error > 0.);
        
        assert(min_inlier_ratio >= 0.);
        
        assert(min_inlier_ratio <= 1.);
        
        assert(confidence >= 0.);
        
        assert(confidence <= 1.0);    
        
        assert(min_num_trials <= max_num_trials);
    }
};

template <typename Estimator, typename SupportMeasurer = InlierSupportMeasurer,
          typename Sampler = RandomSampler>
class RANSAC {
public:
    struct Result{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // the model use eigen 

        bool success = false;

        size_t num_trails = 0;

        /* have the statistics of the inlier num */
        typename SupportMeasurer::Support support;
    
        /* using this a filter */
        std::vector<char> inlier_mask;

        /* the tranform we want to solve */
        typename Estimator::M_t model;
    };

    explicit RANSAC(const RANSACOptions& options);

    // NOTE: we use static here
    /* A static member function can be called even if 
    no objects of the class exist and the static functions are accessed 
    using only the class name and the scope resolution operator ::.*/
    static size_t ComputeNumTrials(
        const size_t num_inliers, // The number of inliers.
        const size_t num_samples, // The total number of samples.
        const double confidence, // Confidence that one sample is outlier-free.
        const double num_trials_multiplier); //Multiplication factor to the computed number of trials.


    Result Estimate(
        const std::vector<typename Estimator::X_t>& X, //Independent variables.
        const std::vector<typename Estimator::Y_t>& Y);//Dependent variables.

    Estimator estimator;
    Sampler sampler;
    SupportMeasurer support_measurer;

protected:
    RANSACOptions options_;
};

//------------------------IMPLEMENT--------------------------
template <typename Estimator, typename SupportMeasurer, typename Sampler>
RANSAC<Estimator, SupportMeasurer, Sampler>::RANSAC(
    const RANSACOptions& options)
    : sampler(Sampler(Estimator::kMinNumSamples)), options_(options) {
  
    options.Check();

    // Determine max_num_trials based on assumed `min_inlier_ratio`.
    const size_t kNumSamples = 100000; // this value should be loaded by Estimator
  
    /* calc the sample num */
    const size_t dyn_max_num_trials = ComputeNumTrials(
        static_cast<size_t>(options_.min_inlier_ratio * kNumSamples),
        kNumSamples,
        options_.confidence, 
        options_.dyn_num_trials_multiplier);

    /* choose it based on the option */
    options_.max_num_trials =
        std::min<size_t>(options_.max_num_trials, dyn_max_num_trials);
}

/*
    CALC THE ITERATION NUM  N
    p : confidence == probability
    in : inlier ratio = num_inliers / num_samples
    s : min sample num to estimate
    1 - p = (1 - in^s)^N
*/
template <typename Estimator, typename SupportMeasurer, typename Sampler>
size_t RANSAC<Estimator, SupportMeasurer, Sampler>::ComputeNumTrials(
    const size_t num_inliers, 
    const size_t num_samples, 
    const double confidence,  // the probability that one sample is outlier-free.
    const double num_trials_multiplier) {

    const double inlier_ratio = num_inliers / static_cast<double>(num_samples);

    const double nom = 1 - confidence;
    if (nom <= 0) {
        return std::numeric_limits<size_t>::max();
    }

    const double denom = 1 - std::pow(inlier_ratio, Estimator::kMinNumSamples);
    if (denom <= 0) {
        return 1;
    }

    return static_cast<size_t>(
        std::ceil(std::log(nom) / std::log(denom) * num_trials_multiplier));
}

/*

*/
template <typename Estimator, typename SupportMeasurer, typename Sampler>
typename RANSAC<Estimator, SupportMeasurer, Sampler>::Report
RANSAC<Estimator, SupportMeasurer, Sampler>::Estimate(
    const std::vector<typename Estimator::X_t>& X,
    const std::vector<typename Estimator::Y_t>& Y) {

    assert(X.size(), Y.size());

    Result res;
    res.success = false;
    res.num_trials = 0;

    if (num_samples < Estimator::kMinNumSamples) {
        return res;
    }

    typename SupportMeasurer::Support best_support;
    typename Estimator::M_t best_model;

    // CTRL LOOP
    // bool abort = false;

    const double max_residual = 
        options_.max_error * options_.max_error;

    // The same size as the X_t, Y_t
    std::vector<double> residuals(num_samples);

    // Values from the sampler
    std::vector<typename Estimator::X_t> X_rand(Estimator::kMinNumSamples);
    std::vector<typename Estimator::Y_t> Y_rand(Estimator::kMinNumSamples);

    // ---1. set the sampler
    sampler.Initialize(num_samples);

    // ---2. find the max iteratio num, mainly based on the manual setting
    size_t max_num_trials = options_.max_num_trials;
    max_num_trials = 
        std::min<size_t>(max_num_trials, sampler.MaxNumSamples());
    
    // ---3. main loop
    for (res.num_trials = 0; res.num_trials < max_num_trials; ++res.num_trials) {

        // --- 3.1 sampling
        sampler.SampleXY(X, Y, &X_rand, &Y_rand);

        // ---3.2 get the estimation
        const std::vector<typename Estimator::M_t> sample_models =
            estimator.Estimate(X_rand, Y_rand);

        //---3.3 Iterate through all estimated models.
        for (const auto& sample_model : sample_models) {
            
            // ---3.3.1 calc residuals
            estimator.Residuals(X, Y, sample_model, &residuals);
            assert(residual.size() == X.size())

            // ---3.3.2 the support to judge the residual
            const auto support = 
                support_measurer.Evaluate(residuals, max_residual);

            // ---3.3.3  Save as best subset if better than all previous subsets.
            if (support_measurer.Compare(support, best_support)) {
                best_support = support;
                best_model = sample_model;

                dyn_max_num_trials = ComputeNumTrials(
                    best_support.num_inliers, 
                    num_samples, 
                    options_.confidence,
                    options_.dyn_num_trials_multiplier);
            }

            // ---3.3.4 if abort
            if (res.num_trials >= dyn_max_num_trials &&
                res.num_trials >= options_.min_num_trials) {
                // abort = true;
                break;
            }

        }// select models
    }// trails


    // ---4. set the result
    res.support = best_support;
    res.model = best_model;

    // --- 4.1 No valid model was found.
    if (report.support.num_inliers < estimator.kMinNumSamples) {
        return report;
    }

    estimator.Residuals(X, Y, report.model, &residuals);
    assert(residuals.size() == X.size());

    report.inlier_mask.resize(num_samples);
    for (size_t i = 0; i < residuals.size(); ++i) {
        if (residuals[i] <= max_residual) {
            res.inlier_mask[i] = true;
        } 
        else {
            res.inlier_mask[i] = false;
        }
    }

    return report;
}


}// namespace
#endif