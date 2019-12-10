/**************************
 * FFeature.cpp
 * ***********************/

#include <opencv2/core/core.hpp>

struct FFeature
{
    FFeature() : idx(0), stat(0), err(0.0), mvec(cv::Point2f(0.0,0.0)), len(0.0) {}

    FFeature(int _idx, unsigned char _stat)
    {
        idx = _idx;
        stat = _stat;
        err = 0.0;
        mvec = cv::Point2f(0.0,0.0);
        len = 0.0;
    }

    int idx; // index of feature in m_keypoints vector
    unsigned char stat; // status of features 1: match, 0: no-match
    float err; // error of the optical flow using lucas-kanade iteration
    cv::Point2f mvec; // summed up motion vector (!constructed from multiple single vectors, where the directions might differ)
    float len; // Length of motion vector

};

struct FFeatureMvecComparator
{
    // Sorting by euclidean-norm-squared of the motion vector
    bool operator() (const FFeature& f1, const FFeature& f2) const
    {
        // Compare squared euclidean distance of the motion vectors
        return (f1.mvec.x * f1.mvec.x + f1.mvec.y * f1.mvec.y) < (f2.mvec.x * f2.mvec.x + f2.mvec.y * f2.mvec.y);
    }
};

struct FFeatureLengthComparator
{
    // Sorting by length of trajectory of the feature motion
    bool operator() (const FFeature& f1, const FFeature& f2) const
    {
        // Compare squared euclidean distance of the motion vectors
        return f1.len < f2.len;
    }
};

struct FFeatureStatComparator
{
    // Sorting by status of match/non-match 
    bool operator() (const FFeature& f1, const FFeature& f2) const
    {
        return f1.stat > f2.stat;
    }
};

struct FFeatureErrComparator
{
    // Sorting by status of match/non-match 
    bool operator() (const FFeature& f1, const FFeature& f2) const
    {
        return f1.err < f2.err;
    }
};
