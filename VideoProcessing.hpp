/*********************************
 * Header file: VideoProcessing.hpp
 *
 * ******************************/

// C++ std libraries
#include <string>
#include <vector>

// User libraries
#include "VideoFrame.hpp"
#include "FeatureTracking.hpp"
#include "VideoStabilizing.hpp"
#include "FeatureTrackingParams.hpp"

class VideoProcessing 
{

    // File path
    std::string m_filePath;

    // File name
    std::string m_fileName;

    // Video capture
    cv::VideoCapture m_videoCapture;
 
    // Frame index of starting frame
    int m_startFrame;
   
    // Number of frames to be processed
    int m_numFrames;

    // Frames
    VideoFrame m_refFrame;

    // Keypoints of all frames
    std::vector<std::vector<cv::Point2f> > m_keypoints;
    
    // Best feature indices
    std::vector<int> m_bestFeatures;

    // Feature tracking algorithm to track features and stabilize frames
    FeatureTracking m_featureTracking;

    // Averaged frames
    cv::Mat m_avgFrame;

    // Alpha masks
    std::vector<cv::Mat> m_alphaMasks;

    public:
    
        // Constructor
        VideoProcessing(const std::string&, const std::string&);

    private:

        // Find feature motion
        void findFeatureMotion();

        // Stabilize frame based on knowledge of feature motion
        void stabilizeFrames(cv::Mat&);

        // Average frames
        void averagingFrames();

        // Create alpha mask of motion
        void createAlphaMask(int, int);        
        
        // Motion blur
        void motionBlur(int, int, const cv::Point2f);
        
        // Open video stream
        bool openVideo(const std::string&);

        // Close video stream
        void closeVideo();

        // Jump to a certain frame number
        bool jumpToFrame(int);

};
