/* ****************************
 * Author: Andrin Jenal
 * Supervisor: Marcel Lancelle
 * Department:
 * Copyright:
 * File: VideoStabilizing.cpp
 * ****************************/

// C++ std libraries
#include <iostream>
#include <sstream>

// User libraries
#include "VideoStabilizing.hpp"
#include "Drawing.hpp"

// Construct a video warper that processes "frames"
VideoStabilizing::VideoStabilizing()
{
    // Empty constructor
}


// stabilizeUsingHomography is a feature based morphing alorithm, that stabilizes frames using weighted motion vectors of the moving features
void VideoStabilizing::stabilizeUsingMorphing(VideoFrame& refFrame, cv::VideoCapture& vidCapt, int numFrames, std::vector<std::vector<cv::Point2f> >& keypoints, std::vector<int>& bestFeatures, cv::Mat& avgFrame)
{
    
    // Temporary placeholder
    VideoFrame nextFrame;

    std::cout << "start stabilization with frame " << (vidCapt.get(cv::CAP_PROP_POS_FRAMES) + 1) << std::endl;
   
    // Iterate over all frames keypoints
    for (std::vector<std::vector<cv::Point2f> >::iterator it = (keypoints.begin()); it != keypoints.end(); ++it)
    {

        // Update current and next frame
        cv::Mat tmpFrame;
        vidCapt.read(tmpFrame);
        nextFrame = VideoFrame(tmpFrame);

        //Drawing::showImg32f(nextFrame.getFrameData32f());
        
        std::ostringstream ostr;
        ostr << "raw/frame" << vidCapt.get(cv::CAP_PROP_POS_FRAMES);

        // For all frames align to reference frame (refFrame)
        nextFrame.alignFrameByFeatureBasedMorphing(refFrame.getKeypoints(), *it, bestFeatures);
        std::cout << "frame " << vidCapt.get(cv::CAP_PROP_POS_FRAMES) << " succesfully warped" << std::endl;
   
        // FOR DEBUGGING PURPOSE ONLY
        Drawing::saveFeatureVecs(refFrame, nextFrame, *it, bestFeatures, ostr.str());

        // FOR ANALYSIS
        ostr << "aligned";
        Drawing::saveImg(nextFrame.getAlignedFrameData32f(), ostr.str());
        ostr << "original";
        Drawing::saveImg(nextFrame.getFrameData32f(), ostr.str());
      
        // Sum up aligned frames to average it afterwards
        avgFrame += nextFrame.getAlignedFrameData32f();
        std::cout << "cummulated frame " << vidCapt.get(cv::CAP_PROP_POS_FRAMES) << std::endl;
    }

    std::cout << "feature based morphing done..." << std::endl;

}
