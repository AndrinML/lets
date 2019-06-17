/*
 *
 */

#include <iostream>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include "Drawing.hpp"

static const std::string kDstFolder = "/tmp/";

// Show keypoints
void Drawing::showKeypoints(const cv::Mat& img, const std::vector<cv::Point2f>& keypoints)
{
    // Copy image source to outImg, such that nothing gets overwritten
    cv::Mat outImg;
    img.copyTo(outImg);
    showImg(drawKeypoints(outImg, keypoints));
}


// Show k best features
void Drawing::showBestFeatures(const cv::Mat& img, const std::vector<cv::Point2f>& keypoints, const std::vector<int>& bestFeatures)
{
    // Copy image source to outImg, such that nothing gets overwritten
    cv::Mat outImg;
    img.copyTo(outImg);
    showImg32f(drawBestFeatures(outImg, keypoints, bestFeatures));
}


// Show feature vectors
void Drawing::showFeatureVecs(VideoFrame& refFrame, VideoFrame& frame2, const std::vector<cv::Point2f>& frame2Keypoints, const std::vector<int>& bestFeatures)
{
    cv::Mat outImg;
    refFrame.getAlignedFrameData32f().copyTo(outImg);
    showImg32f(drawFeatureVecs(refFrame, frame2Keypoints, bestFeatures, outImg));
}


// Show summed up motion vectors
void Drawing::showMotionVecs(VideoFrame& refFrame, const std::vector<std::vector<cv::Point2f> >& keypoints, const std::vector<int>& bestFeatures)
{
    cv::Mat outImg;
    refFrame.getFrameData().copyTo(outImg);
    showImg32f(drawMotionVecs(refFrame, keypoints, bestFeatures, outImg, false));
}


// Show image
void Drawing::showImg(cv::Mat& img)
{
    const std::string winname = "showImg";
    cv::namedWindow(winname, CV_WINDOW_AUTOSIZE);
    cv::imshow(winname, img);
    cv::waitKey(0);
}


// Show image
void Drawing::showImg32f(cv::Mat& img)
{
    // Shifting color range to 0..255
    normalize(img, img, 0, 1, CV_MINMAX);
    const std::string winname = "showImg32f";
    cv::namedWindow(winname, CV_WINDOW_AUTOSIZE);
    cv::imshow(winname, img);
    cv::waitKey(0);
}


// Save image
void Drawing::saveImg(const cv::Mat& img, const std::string fileName)
{
    cv::imwrite(kDstFolder + "/vidstab/images/" + fileName + ".jpg", img);
    std::cout << kDstFolder + "vidstab/images/" + fileName << ".jpg" << " successfully saved..." << std::endl;
}


// Save best features
void Drawing::saveKeypoints(const cv::Mat& img, const std::vector<cv::Point2f>& keypoints, const std::vector<int>& bestFeatures, const std::string& fileName)
{
    cv::Mat outImg;
    img.copyTo(outImg);
    saveImg(drawKeypoints(outImg, keypoints, bestFeatures), fileName);
}


// Save best features
void Drawing::saveBestFeatures(const cv::Mat& img, const std::vector<cv::Point2f>& keypoints, const std::vector<int>& bestFeatures, const std::string& fileName)
{
    cv::Mat outImg;
    img.copyTo(outImg);
    saveImg(drawBestFeatures(outImg, keypoints, bestFeatures), fileName);
}


// Save feature vectors
void Drawing::saveFeatureVecs(VideoFrame& frame1, VideoFrame& frame2, const std::vector<cv::Point2f>& frame2Keypoints, const std::vector<int>& bestFeatures, const std::string& fileName)
{
    cv::Mat outImg;
    frame2.getAlignedFrameData32f().copyTo(outImg);
    saveImg(drawFeatureVecs(frame1, frame2Keypoints, bestFeatures, outImg), fileName);
}


// Save motion vectors (feature trajectories)
void Drawing::saveMotionVecs(VideoFrame& refFrame, const std::vector<std::vector<cv::Point2f> >& keypoints, const std::vector<int>& bestFeatures, bool showMvecs, const std::string& fileName)
{
    cv::Mat outImg;
    refFrame.getFrameData().copyTo(outImg);
    saveImg(drawMotionVecs(refFrame, keypoints, bestFeatures, outImg, showMvecs), fileName);
}


// Draw keypoints
cv::Mat& Drawing::drawKeypoints(cv::Mat& img, const std::vector<cv::Point2f>& keypoints)
{
    // Iterate over all keypoints
    for (int i = 0; i < keypoints.size(); ++i)
    {
        // Color keypoints of the current frame
        cv::circle(img, keypoints[i], 2, cv::Scalar(0,0,255), 1, 8, 0);
    }

    return img;
}


// Draw keypoints distinguish best features
cv::Mat& Drawing::drawKeypoints(cv::Mat& img, const std::vector<cv::Point2f>& keypoints, const std::vector<int>& bestFeatures)
{
    // Iterate over all keypoints
    for (int i = 0; i < keypoints.size(); ++i)
    {
        
        // If feature i was used for the computation then the color it red otherwise green
        if (bestFeatures.end() != std::find(bestFeatures.begin(), bestFeatures.end(), i))
        {

            // Color keypoints of the current frame
            cv::circle(img, keypoints[i], 2, cv::Scalar(0,0,255), 3, 8, 0);

        } else {

            // Color keypoints of the current frame
            cv::circle(img, keypoints[i], 2, cv::Scalar(0,255,0), 3, 8, 0);

        }

    }

    return img;
}



// Draw k best features of two consecutive frames
cv::Mat& Drawing::drawBestFeatures(cv::Mat& img, const std::vector<cv::Point2f>& keypoints, const std::vector<int>& bestFeatures)
{
    // Iterate over best features in that frame
    for (int i = 0; i < bestFeatures.size(); ++i)
    {
        // Color keypoints of the current frame (this)
        cv::circle(img, keypoints[bestFeatures[i]], 2, cv::Scalar(0,0,255), 1, 8, 0);
    }

    return img;
}


// Draw feature vectors between two frames
// @frame1       first frame
// @frame2       second frame
// @bestFeatures container of best features
// @k            number of used keypoints
cv::Mat& Drawing::drawFeatureVecs(VideoFrame& refFrame, const std::vector<cv::Point2f>& frame2Keypoints, const std::vector<int>& bestFeatures, cv::Mat& outImg)
{
    // Get keypoints and feature data of the reference and current frame
    std::vector<cv::Point2f> refFrameKeypoints = refFrame.getKeypoints();
    
    for (int i = 0; i < refFrameKeypoints.size(); ++i)
    {
   
        // Extract position p(x,y) of keypoints
        cv::Point2f p1 = refFrameKeypoints[i];
        cv::Point2f p2 = frame2Keypoints[i];

        // If feature i was used for the computation then the color it red otherwise green
        if (bestFeatures.end() != std::find(bestFeatures.begin(), bestFeatures.end(), i))
        {
            // Color red
            cv::line(outImg, p1, p1 + (p2-p1), cv::Scalar(0,0,255), 1, 8, 0);
        } else {

            // Color green
            cv::line(outImg, p1, p1 + (p2-p1), cv::Scalar(0,255,0), 1, 8, 0);
        }
    }

    return outImg;
}


// Draw motion vectors between start and end frame
// @frames       all frames
// @startFrame   starting frame
// @bestFeatures container of best features
// @k            number of used keypoints
cv::Mat& Drawing::drawMotionVecs(VideoFrame& refFrame, const std::vector<std::vector<cv::Point2f> >& keypoints, const std::vector<int>& bestFeatures, cv::Mat& outImg, bool mvecs)
{
    // Draw circles on the first to indiciate feature detection
    drawBestFeatures(outImg, keypoints[0], bestFeatures);

    // Iterate over all frames and draw feature motions
    for (int f = 0; f < keypoints.size()-1; ++f)
    {

        for (int i = 0; i < keypoints[f].size(); ++i)
        {
            // Extract position p(x,y) of keypoints
            cv::Point2f p1 = keypoints[f][i];
            cv::Point2f p2 = keypoints[f+1][i];

            // If feature i was used for the computation then the color it red otherwise green
            if (bestFeatures.end() != std::find(bestFeatures.begin(), bestFeatures.end(), i))
            {
                // Color red
                cv::line(outImg, p1, p1 + (p2-p1), cv::Scalar(0,0,255), 1, 8, 0);
            } else {

                // Color green
                cv::line(outImg, p1, p1 + (p2-p1), cv::Scalar(0,255,0), 1, 8, 0);
            }
        }
    }


    /*
    if (mvecs)
    {
        // Draw the mvecs for all features
        for (int i = 0; i < frames[endFrame].getFeatureData().size(); ++i)
        {
            // Color mvecs blue
            cv::Point2f p = keypoints[frames[endFrame].getFeatureData()[i].idx];
            cv::Point2f mvec = frames[endFrame].getFeatureData()[i].mvec;
            cv::line(outImg, p, p + mvec, cv::Scalar(255,0,0), 1, 8, 0);
        }
    }
    */

    return outImg;
}

