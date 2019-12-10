/*
 *
 */

#ifndef VIDEOSTAB_DRAWING_HPP
#define VIDEOSTAB_DRAWING_HPP

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "VideoFrame.hpp"

class Drawing
{
    public:

        static void showKeypoints(const cv::Mat&, const std::vector<cv::Point2f>&);

        static void showBestFeatures(const cv::Mat&, const std::vector<cv::Point2f>&, const std::vector<int>&);
        
        static void showFeatureVecs(VideoFrame&, VideoFrame&, const std::vector<cv::Point2f>&, const std::vector<int>&);

        static void showMotionVecs(VideoFrame&, const std::vector<std::vector<cv::Point2f> >&, const std::vector<int>&);

        static void showImg(cv::Mat&);

        static void showImg32f(cv::Mat&);

        static void saveImg(const cv::Mat&, const std::string);
        
        static void saveBestFeatures(const cv::Mat&, const std::vector<cv::Point2f>&, const std::vector<int>&, const std::string&);

        static void saveKeypoints(const cv::Mat&, const std::vector<cv::Point2f>&, const std::vector<int>&, const std::string&);

        static void saveFeatureVecs(VideoFrame&, VideoFrame&, const std::vector<cv::Point2f>&, const std::vector<int>&, const std::string&);

        static void saveMotionVecs(VideoFrame&, const std::vector<std::vector<cv::Point2f> >&, const std::vector<int>&, bool, const std::string&);

        static cv::Mat& drawKeypoints(cv::Mat&, const std::vector<cv::Point2f>&);

        static cv::Mat& drawKeypoints(cv::Mat&, const std::vector<cv::Point2f>&, const std::vector<int>&);

        static cv::Mat& drawBestFeatures(cv::Mat&, const std::vector<cv::Point2f>&, const std::vector<int>&);

        static cv::Mat& drawFeatureVecs(VideoFrame&, const std::vector<cv::Point2f>&, const std::vector<int>&, cv::Mat&);
    
        static cv::Mat& drawMotionVecs(VideoFrame&, const std::vector<std::vector<cv::Point2f> >&,const std::vector<int>&, cv::Mat&, bool);

};

#endif // VIDEOSTAB_DRAWING_HPP
