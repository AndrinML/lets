/**********************
 * VideoFrame.hpp
 * *******************/

#ifndef VIDEOSTAB_FRAME_HPP
#define VIDEOSTAB_FRAME_HPP

// C++ std libraries
#include <string>
#include <vector>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// User libraries
#include "FeatureTrackingParams.hpp"
#include "FFeature.cpp"

class VideoFrame
{
     
    // cv::Mat container for the frame data
    cv::Mat m_frameData;

    // cv::Mat container for the frame data
    cv::Mat m_frameData32f;

    // cv::Mat container for aligned frame data of type CV_32FC3
    cv::Mat m_alignedFrameData32f;

    // Container for keypoints
    std::vector<cv::Point2f> m_keypoints;

    // Container for feature status
    std::vector<unsigned char> m_status;

    // Container for possible feature matching error
    std::vector<float> m_error;

    // Sorted container for the feature data
    std::vector<FFeature> m_featureData;

    public:

        // Empty constructor
        VideoFrame() {};
        // Constructor takes a frame as input
        VideoFrame(cv::Mat&);

        // Refine keypoints on search domain
        int refineGoodFeatures(FeatureTrackingParams, int[]);

        // Calculate sparse optical flow between i-th and i+1-th frame
        void calcOpticalFlow(VideoFrame&);

        // Find best features based on the cummulated erros & return mean motion vector
        void findBestFeatures(std::vector<int>&);

        // Aligns frame i to frame i-1 (previous)
        void alignFrameByFeatureBasedMorphing(bool, cv::Mat, const std::vector<cv::Point2f>&, std::vector<cv::Point2f>&, std::vector<int>&);

        // Pixel look up with boundary check
        cv::Vec3f getPixelAt(int, int);
        

        // Getter
        // Return frame data
        cv::Mat& getFrameData();

        // Return frame data 32 float format
        cv::Mat& getFrameData32f();

        // Return aligned frame data 32 float format
        cv::Mat& getAlignedFrameData32f();

        // Return feature data
        std::vector<FFeature>& getFeatureData();

        // Return keypoints
        std::vector<cv::Point2f>& getKeypoints();

        // Return status vector
        std::vector<unsigned char>& getStatusVec();

    private:

        // Euclidean distance of vector
        float euclDist(cv::Point2f);

        // Linearly interpolate look up pixels
        // four neighbourhood interpolation
        cv::Vec3f interpolatedPixelLookUp(float, float);
        
        // Weighting function need by feature based morphing algorithm
        float weightFunction(float);
};

#endif // VIDEOSTAB_FRAME_HPP
