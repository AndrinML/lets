/**************************************
 * Header file: VideoStabilizing.hpp
 * 
 * Defines all methods for stabilizing
 * (warping or morphing) the video
 *
 * ***********************************/

#ifndef VIDEOSTAB_VIDEOSTABILIZING_HPP
#define VIDEOSTAB_VIDEOSTABILIZING_HPP 

// C++ std libraries
#include <string>
#include <vector>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// User libraries
#include "VideoFrame.hpp"

class VideoStabilizing 
{

    public:
        
        // Constructors
        VideoStabilizing();

        // Feature based morphing
        void stabilizeUsingMorphing(VideoFrame&, cv::VideoCapture&, int, std::vector<std::vector<cv::Point2f> >&, std::vector<int>&, cv::Mat&);

};

#endif // VIDEOSTAB_VIDEOSTABILIZING_HPP
