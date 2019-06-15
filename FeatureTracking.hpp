/****************************
 * Header file: FeatureTracking.hpp
 *
 * **************************/

#ifndef VIDEOSTAB_FEATURETRACKING_HPP
#define VIDEOSTAB_FEATURETRACKING_HPP

#include <string>
#include <vector>
#include "VideoFrame.hpp"

class FeatureTracking
{
   
    // File name
    const std::string& m_fileName;

    // Feature tracking parameters
    FeatureTrackingParams m_ftParams;

    public:

        // Constructor
        FeatureTracking(const std::string&);

        int computeGoodFeatures(VideoFrame&);

        int refineGoodFeatures(VideoFrame&, std::vector<int>&);

        void initialMotion(VideoFrame&, cv::VideoCapture&, int, std::vector<int>&);

        void refinedMotion(VideoFrame&, cv::VideoCapture&, int, std::vector<int>&, std::vector<std::vector<cv::Point2f> >&);

};

#endif // VIDEOSTAB_FEATURETRACKING_HPP
