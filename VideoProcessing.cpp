/* ***********************************
 * Author: Andrin Jenal
 * Supervisor: Marcel Lancelle
 * Department: ETH Zürich
 * Copyright: 2013 ETH Zürich
 * File: VideoProcessing.cpp
 * **********************************/

// C++ std libraries
#include <iostream>
#include <sstream>

// User libraries
#include "VideoProcessing.hpp"
#include "VideoFrame.hpp"
#include "Drawing.hpp"
#include "Timer.hpp"

// Constructor
VideoProcessing::VideoProcessing(const std::string& videoFilePath, const std::string& videoName) : m_featureTracking(videoName), m_videoCapture(videoFilePath) 
{
    // Declare and start timer
    Timer timer;
    timer.start();

    m_filePath = videoFilePath;
    
    // File name to which video gets saved
    m_fileName = videoName;

    // Define number of frames to work with
    // The averaged images contains m_numFrames + (reference Frame) frames
    m_numFrames = 30;
   
    std::cout << "start computation with: " << m_numFrames << " frames" << std::endl;

    // Find feature motion that is later used for optical flow computation and video stabilization
    findFeatureMotion();

    Drawing::saveMotionVecs(m_refFrame, m_keypoints, m_bestFeatures, false, m_fileName + "_motionVecs");
    
    // Stabilize frames 
    stabilizeFrames(m_avgFrame);

    // Average over all aligned frames
    averagingFrames();

    // Create alpha mask for global motion 
    //createAlphaMask(startFrame, endFrame);
    //motionBlur(startFrame, endFrame, cv::Point2f(0.0, 0.0));

    // Stop timer and print time
    double elapsedTime = timer.stop();
    std::cout << "computational time: " << elapsedTime << " seconds" << std::endl;
    
}


// Find feature motion
void VideoProcessing::findFeatureMotion()
{
    // Open video capture
    openVideo(m_filePath); 

    // Initialize m_keypoints vector for all keypoints for all frames
    m_keypoints = std::vector<std::vector<cv::Point2f> >(m_numFrames);
    
    int frameCount = m_videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
    m_startFrame = frameCount/2 - m_numFrames/2;

    // Jump to start frame index - 1
    jumpToFrame(m_startFrame - 1);

    // Create first video frame (reference frame)
    cv::Mat tmpFrame;
    m_videoCapture.read(tmpFrame);
    m_refFrame = VideoFrame(tmpFrame);

    // Compute good features on reference frame
    // @domainSplit enabled
    int numFeats = m_featureTracking.computeGoodFeatures(m_refFrame);
    std::cout << numFeats << " good features detected in reference frame " << m_startFrame << std::endl;

    // Optical flow calculation
    m_featureTracking.initialMotion(m_refFrame, m_videoCapture, m_numFrames, m_bestFeatures);
    
    // Close video stream
    closeVideo();    
    
    // Open video
    openVideo(m_filePath);

    // Jump to start frame index - 1
    jumpToFrame(m_startFrame);

    // Compute good features on reference frame on subdomain
    numFeats = m_featureTracking.refineGoodFeatures(m_refFrame, m_bestFeatures);
    std::cout << numFeats << " good features detected in reference frame " << m_startFrame << std::endl;

    // Refined optical flow calculation on a subdomain of the original frame
    m_featureTracking.refinedMotion(m_refFrame, m_videoCapture, m_numFrames, m_bestFeatures, m_keypoints);

    // Close video stream
    closeVideo();
}

// Video (frame) stabilization
// Always start from startFrame = startFrame + 1 to align current to previous frame
void VideoProcessing::stabilizeFrames(cv::Mat& avgFrame)
{

    // Prepare for averaging
    // Add reference frame to the avgFrame image
    m_refFrame.getFrameData32f().copyTo(avgFrame);

    // Open the video stream
    openVideo(m_filePath);

    // Jump to reference frame index
    jumpToFrame(m_startFrame);

    // Construct and initialize the video stabilizing object
    // Warp all frames to the reference frame 
    // Start stabilizing from the subsequent frame
    VideoStabilizing vidStab = VideoStabilizing();
   
    // Perform video stabilization
    vidStab.stabilizeUsingMorphing(m_refFrame, m_videoCapture, m_numFrames, m_keypoints, m_bestFeatures, avgFrame);
    
    std::cout << "video stabilization done..." << std::endl;

}

// Averaging aligned frames
void VideoProcessing::averagingFrames()
{

    // Divide by number of frames
    m_avgFrame *= (1.0/((m_numFrames + 1)));

    std::cout << "averaging done..." << std::endl;

    Drawing::saveImg(m_avgFrame, m_fileName + "_avg");

}


// Open the video stream
bool VideoProcessing::openVideo(const std::string& filePath)
{
    if (!m_videoCapture.open(filePath))
    {
        std::cout << "Cannot open the video" << std::endl;
    }
    else
    {
        int frameCount = m_videoCapture.get(CV_CAP_PROP_FRAME_COUNT);   
        std::cout << "Video " << m_fileName << " successfully opened. " << frameCount << " frames loadable." << std::endl;
    }
}


// Close the video stream
void VideoProcessing::closeVideo()
{
    m_videoCapture.release();
}


// Jump to a certain frame number
bool VideoProcessing::jumpToFrame(int pos)
{
    for (int idx = 0; idx < pos; ++idx)
    {
        m_videoCapture.grab();
    }

    std::cout << "jumped to " << m_videoCapture.get(CV_CAP_PROP_POS_FRAMES) << std::endl;
}


// MAIN
int main (int argc, char** argv)
{
    std::string file;
    std::string fileName;
    std::string fileType;

    if (argc > 1)
    {
        file = argv[1];
        fileName = file.substr(0, file.size() - 4);
        fileType = file.substr(file.size() - 4, file.size());
    }
    else
    {
        fileName = "mini_waterfall";
        fileType = ".avi";
    }
    
    VideoProcessing VidProc("../videos/non_stabilized_samples/" + fileName + fileType, fileName);

    return 0;
}
