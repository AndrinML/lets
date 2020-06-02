/* ***********************************
 * Author: Andrin Jenal
 * Supervisor: Marcel Lancelle
 * Department: ETH Zürich
 * Copyright: 2013 ETH Zürich
 * File: FeatureTracking.cpp
 * **********************************/

// C++ std libraries
#include <iostream>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// User libraries
#include "FeatureTracking.hpp"
#include "FeatureTrackingParams.hpp"
#include "Drawing.hpp"


// Constructor
FeatureTracking::FeatureTracking(const std::string& fileName) : m_fileName(fileName)
{
    
    // Params for cv::goodFeaturesToTrack
    // @maxNumFeatures:     Maximum number of features that will be returned
    // @qualityLevel:       Parameter for minimal accepted quality of image corners
    // @minDistance:        Minimum possible euclidean distance between returned corners
    // @blockSize:          Size of an average block for computing a derivative covariation matrix 
    // @useHarrisDetector:  By default "false"
    m_ftParams.maxNumFeat = 240;
    m_ftParams.qualLev = 0.01;
    m_ftParams.minDist = 1.0;
    m_ftParams.blSize = 3;
    m_ftParams.harrCor = false;

}


// Compute good features in the current frame, proceed by domain decomposition to find uniformely distributed features
// @maxNumFeatures: max features that should be found with given quality level
// @domainSplit: split domain in subdomains, default = 0
int FeatureTracking::computeGoodFeatures(VideoFrame& vidFrame)
{

    // One could improve the found features by accepting only a certain number of features in a certain radius -> better distribution of good features

    // Convert frame to grayscale image
    cv::Mat greyScaleFrameData;
    cv::cvtColor(vidFrame.getFrameData(), greyScaleFrameData, cv::COLOR_RGB2GRAY);

    // Constant number of subdomains
    int xStep = 4;
    int yStep = 3;
    int numSubDoms = xStep * yStep;
    
    // Calculate the number of features in each subdomain
    int numFeatsDom = (int) (m_ftParams.maxNumFeat / numSubDoms);
  
    // Initialize temp vector to store intermediate results (keypoints)
    std::vector<cv::Point2f> tmpKeypts;
    tmpKeypts.resize(numFeatsDom);

    // Create mask container for region of interest
    float frameWidth = vidFrame.getFrameData32f().size().width;
    float frameHeight = vidFrame.getFrameData32f().size().height;


    int segWidth = (int) (frameWidth / xStep);
    int segHeight = (int) (frameHeight / yStep);

    for (int dy = 0; dy < yStep; ++dy)
    {
        for (int dx = 0; dx < xStep; ++dx)
        {

            // Decompose domain into numSubDoms segments
            // This is a static approach that supposes aspect ration 4:3
            int x = dx * segWidth;
            int y = dy * segHeight;

            // Create mask from segment data
            cv::Mat subDomainMask = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
            subDomainMask(cv::Rect(x, y, segWidth, segHeight)) = (unsigned char) 255;
            
            // Find good features to track using segment mask
            cv::goodFeaturesToTrack(greyScaleFrameData, tmpKeypts, numFeatsDom, m_ftParams.qualLev, m_ftParams.minDist, subDomainMask, m_ftParams.blSize, m_ftParams.harrCor);
            vidFrame.getKeypoints().insert(vidFrame.getKeypoints().end(), tmpKeypts.begin(), tmpKeypts.end());

        }
    }

    // Initialize feature data structure to save motion vectors
    vidFrame.getFeatureData() = std::vector<FFeature>(vidFrame.getKeypoints().size());

    // Return number of accepted features
    return vidFrame.getKeypoints().size();
}


// refineGoodFeatures: refine good features to track on sub-range of original frame
int FeatureTracking::refineGoodFeatures(VideoFrame& vidFrame, std::vector<int>& bestFeatures)
{

    // Find range of best features
    // @range: minX = range[0], maxX = range[1], minY = range[2], maxY = range[3]
    int range[] = {99999, -99999, 99999, -99999};

    std::vector<cv::Point2f> keypoints = vidFrame.getKeypoints();
    
    for (int i = 0; i < bestFeatures.size(); ++i)
    {
       
        if (keypoints[bestFeatures[i]].x < range[0])
        {
            range[0] = keypoints[bestFeatures[i]].x;
        }
        if (keypoints[bestFeatures[i]].x > range[1])
        {
            range[1] = keypoints[bestFeatures[i]].x;
        }
        if (keypoints[bestFeatures[i]].y < range[2])
        {
            range[2] = keypoints[bestFeatures[i]].y;
        }
        if (keypoints[bestFeatures[i]].y > range[3])
        {
            range[3] = keypoints[bestFeatures[i]].y;
        }
    }

    // HARDCODDED RANGE - USER SPECIFIED REGION OF INTEREST
   
    /*
    range[0] = 440.0;
    range[1] = 620.0;
    range[2] = 470.0;
    range[3] = 540.0;
    */

    // Notify when frame succesfully processed
    std::cout << "range: x: " << range[0] << " - " << range[1] << ", y: " << range[2] << " - " << range[3] << std::endl;
    // Convert frame to grayscale image
    cv::Mat greyScaleFrameData;
    cv::cvtColor(vidFrame.getFrameData32f(), greyScaleFrameData, cv::COLOR_RGB2GRAY);
 
    // @range: minX = range[0], maxX = range[1], minY = range[2], maxY = range[3]
    cv::Mat mask = cv::Mat::zeros(greyScaleFrameData.size(), CV_8UC1);
    mask(cv::Rect(range[0],range[2],range[1]-range[0],range[3]-range[2])) = (unsigned char) 255;

    // Detect good features using cv::goodFeaturesToTrack with mask
    cv::goodFeaturesToTrack(greyScaleFrameData, vidFrame.getKeypoints(), m_ftParams.maxNumFeat, m_ftParams.qualLev, m_ftParams.minDist, mask, m_ftParams.blSize, m_ftParams.harrCor);    

    // Resize feature data vector
    vidFrame.getFeatureData().resize(vidFrame.getKeypoints().size());

    return vidFrame.getKeypoints().size();
}


// Processed by KLT
// initialMotion: calculate the initial feature motion
// @return: number of good features to track
void FeatureTracking::initialMotion(VideoFrame& refFrame, cv::VideoCapture& vidCapt, int numFrames, std::vector<int>& bestFeatures)
{
    
    // Temporary placeholders 
    VideoFrame currFrame;
    VideoFrame nextFrame = refFrame;

    // Track initial features over remaining video frames
    for (int i = 0; i < numFrames; ++i)
    {

        // Update current and next frame
        currFrame = nextFrame;
        cv::Mat tmpFrame;
        vidCapt.read(tmpFrame);
        nextFrame = VideoFrame(tmpFrame);
        
        // Calculate optical flow of features between frames
        currFrame.calcOpticalFlow(nextFrame);

        // Copy computed keypoints into the 'global' keypoints container
        //keypoints[i] = nextFrame.getKeypoints();
        
        std::cout << "optical flow between frames " << (vidCapt.get(cv::CAP_PROP_POS_FRAMES) - 1) << " and " << vidCapt.get(cv::CAP_PROP_POS_FRAMES) << " calculated" << std::endl;

    }

    // Find best features based on the cummulated errors in the last frame
    nextFrame.findBestFeatures(bestFeatures);

    std::cout << "initial feature tracking done..." << std::endl;

    Drawing::saveBestFeatures(refFrame.getFrameData(), refFrame.getKeypoints(), bestFeatures, "raw/" + m_fileName + "_FeatureDetection1");
    
    Drawing::saveKeypoints(refFrame.getFrameData(), refFrame.getKeypoints(), bestFeatures, "raw/" + m_fileName + "_FeatureDetection1_Keypoints");
}


// Calculate refined feature motion based a range analysis
void FeatureTracking::refinedMotion(VideoFrame& refFrame, cv::VideoCapture& vidCapt, int numFrames, std::vector<int>& bestFeatures, std::vector<std::vector<cv::Point2f> >& keypoints)
{

    // Temporary placeholders 
    VideoFrame currFrame;
    VideoFrame nextFrame = refFrame;
    
    // Track initial features over remaining video frames
    for (int i = 0; i < numFrames; ++i)
    {
        
        // Update current and next frame
        currFrame = nextFrame;
        cv::Mat tmpFrame;
        vidCapt.read(tmpFrame);
        nextFrame = VideoFrame(tmpFrame);

        // Calculate optical flow of features between frames
        currFrame.calcOpticalFlow(nextFrame);

        // Copy computed keypoints into the 'global' keypoints container
        keypoints[i] = nextFrame.getKeypoints();

        std::cout << "optical flow between frames " << (vidCapt.get(cv::CAP_PROP_POS_FRAMES) - 1) << " and " << vidCapt.get(cv::CAP_PROP_POS_FRAMES) << " calculated" << std::endl;
    }

    // Find best features based on the cummulated errors in the last frame
    nextFrame.findBestFeatures(bestFeatures);

    std::cout << "refined feature tracking done..." << std::endl;

    Drawing::saveBestFeatures(refFrame.getFrameData(), refFrame.getKeypoints(), bestFeatures, "raw/" + m_fileName + "_FeatureDetection2");

    Drawing::saveKeypoints(refFrame.getFrameData(), refFrame.getKeypoints(), bestFeatures, "raw/" + m_fileName + "_FeatureDetection2_Keypoints");
}
