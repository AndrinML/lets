/************************
 * VideoFrame.cpp
 * *********************/

// C++ std libraries
#include <iostream>
#include <algorithm>
#include <utility>

// OpenCV libraries
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>

// User libraries
#include "VideoFrame.hpp"
#include "Drawing.hpp"

// Constructor: (called in VideoData)
VideoFrame::VideoFrame(cv::Mat& frame)
{

    // m_frameData stores all frame data in CV_8UC3 format
    frame.copyTo(m_frameData);

    // Temporary container to convert frame type from CV_8UC3 to CV_32FC3
    cv::Mat tmpFrame32f(frame.size(), CV_32FC3);
    
    // Convert frame data to CV_32FC3
    frame.convertTo(tmpFrame32f, CV_32FC3);

    // m_frameData32f stores all frame data in CV_32FC3 format
    tmpFrame32f.copyTo(m_frameData32f);

    // m_alignedFrameData stores transformed frame of different type format
    //tmpFrame32f.copyTo(m_alignedFrameData32f);
    m_alignedFrameData32f = cv::Mat::zeros(tmpFrame32f.size(), tmpFrame32f.type());

    // m_keypoints stores all keypoints 
    m_keypoints = std::vector<cv::Point2f>();

    // m_status stores whether a feature matches or not
    m_status = std::vector<unsigned char>();

    // m_error stores the error of optical flow
    m_error = std::vector<float>();

}


// Calculate sparse optical flow between this and next frame
void VideoFrame::calcOpticalFlow(VideoFrame& nextFrame)
{

    // Convert both frames to grayscale images
    cv::Mat prevImg, nextImg;
    cv::cvtColor(m_frameData, prevImg, CV_RGB2GRAY);
    cv::cvtColor(nextFrame.m_frameData, nextImg, CV_RGB2GRAY);

    // Calculate optical flow using interative Lucas-Kanade method
    cv::calcOpticalFlowPyrLK(prevImg, nextImg, m_keypoints, nextFrame.m_keypoints, nextFrame.m_status, m_error);

    // Initialize total error for the next frame (nextFrame)
    nextFrame.m_featureData = std::vector<FFeature>(nextFrame.m_keypoints.size());

    // For all keypoints (features) sum up motion vector, length and error
    for (int i = 0; i < nextFrame.m_keypoints.size(); ++i)
    {
        // Initialize FFeature struct, FFeature.mvec is initially set to (0,0)
        nextFrame.m_featureData[i] = FFeature(i, 0);

        // Calculate trajector vector (mvec == motion vector) of features
        // If match of two features found then set status to 1
        if (nextFrame.m_status[i] != 0)
        {
            nextFrame.m_featureData[i].stat = 1;
        }
        
        // Sum up motion vector so far (direction)
        nextFrame.m_featureData[i].mvec = m_featureData[i].mvec + (nextFrame.m_keypoints[i] - m_keypoints[i]);
       
        // Update length of trajectory of feature motion so far
        nextFrame.m_featureData[i].len = m_featureData[i].len + euclDist(nextFrame.m_keypoints[i] - m_keypoints[i]); 
        
        // Sum up error of optical flow 
        nextFrame.m_featureData[i].err = m_featureData[i].err + m_error[i];

    }

}


// Find best features (motion vectors with smallest euclidean-norm-squared)
// Usually this method only gets called on the last frame of interest
void VideoFrame::findBestFeatures(std::vector<int>& bestFeatures)
{
    
    // Initialize bestFeatures container
    bestFeatures = std::vector<int>(m_featureData.size());
   
    // Sort by status (1: match, 0: no-match)
    std::sort(m_featureData.begin(), m_featureData.end(), FFeatureStatComparator()); 

    // Find number of matches in m_featureData
    int matchIdx = 0;
    while (m_featureData[matchIdx].stat != 0 && matchIdx < m_featureData.size())
    {
        matchIdx++;
    }

    // Sort feature data by error of lucas-kanade iteration
    std::sort(m_featureData.begin(), m_featureData.begin() + matchIdx, FFeatureErrComparator());

    // Reject errorprone matches (keep 90% of the best matches)
    matchIdx = (int) matchIdx * 0.9;
    
    // Sort feature data by length of trajectory of the feature motion
    // Only consider matches
    std::sort(m_featureData.begin(), m_featureData.begin() + matchIdx, FFeatureLengthComparator());

    // Reject non similar matches (keep 90% of the shotest trajectories)
    matchIdx = (int) matchIdx * 0.9;
    
    // Initialize space bestFeatures vector
    bestFeatures.resize(matchIdx);

    
    // Get indices of best features to track
    for (int i = 0; i < bestFeatures.size(); ++i)
    {
        bestFeatures[i] = m_featureData[i].idx;
    }

    // Feature selection improvement:
    // Select only features that are similar to the mean feature characteristics
    // Meaning similar angle and length (similar radius)
    
    // Define reference vector, in this case the shortest
    // Assuming it belongs to the moving object
    cv::Point2f refMotionVec = m_featureData[0].mvec;
    float refMotionLength = m_featureData[0].len;

    // Vector difference tolerance between reference vector and motion vector
    // TODO These static parameters highly contribute to the quality of the solution
    float radTol = 100.0;
    // Length tolerance
    float lengthTol = 50.0;

    int featureCounter = 0;
    for (int i = 0; i < bestFeatures.size(); ++i)
    {
        float mvecLength = m_featureData[i].len; 
      
        // Calculate distance of mean vector to motion vector
        float xDiff = std::abs(m_featureData[i].mvec.x - refMotionVec.x);
        float yDiff = std::abs(m_featureData[i].mvec.y - refMotionVec.y);

        // Consider feature as "good" if similar length and direction as the mean feature
        if (std::abs(refMotionLength - mvecLength) < lengthTol && (xDiff + yDiff) < radTol)
        {
            bestFeatures[featureCounter] = m_featureData[i].idx;
            ++featureCounter;
        }
    }

    // Resize bestFeatures to the size of "good" features
    bestFeatures.resize(featureCounter);

    std::cout << "keep: " << featureCounter << " best features" << std::endl;

}


// Align two (consecutive) frames to stabilize video
// Using the feature based mapping method
void VideoFrame::alignFrameByFeatureBasedMorphing(const std::vector<cv::Point2f>& refFrameKeypts, std::vector<cv::Point2f>& keypoints, std::vector<int>& bestFeatures)
{

    // Create look up for interpolated weighting function for N sample points
    int N = 500;
    std::vector<float> intpWeights = std::vector<float>(N, 0.0);

    // Maximum distance of two pixels
    float maxDist = std::sqrt(m_frameData32f.size().width * m_frameData32f.size().width + m_frameData32f.size().height * m_frameData32f.size().height);

    // Step width of sample points
    float step = maxDist / N;

    // Interpolated function ranges from [0..maxDist]
    for (int sIdx = 0; sIdx < N; ++sIdx)
    {
        intpWeights[sIdx] = weightFunction(sIdx * step);
    }

    // Iterate over all pixels in the frame (m_frameData)
    for (int i = 0; i < m_frameData32f.size().height; ++i)
    {
        for (int j = 0; j < m_frameData32f.size().width; ++j)
        {
            // Initialize new weight and new lookup vector
            //@totalWeight: summed up weight over all features
            float totalWeight = 0.0;
            cv::Point2f lookupVector = cv::Point2f(0.0,0.0);
            
            // Iterate over all feature and sum up weighted motion vector
            for (int f = 0; f < bestFeatures.size(); ++f)
            {
                // distance to features in reference frame
                //@distance: distance of pixel to specific feature
                cv::Point2f refFtrPos = refFrameKeypts[bestFeatures[f]];
                float absX = (refFtrPos.x - j) * (refFtrPos.x - j);
                float absY = (refFtrPos.y - i) * (refFtrPos.y - i);
                float distance = std::sqrt(absX + absY);
               
                // Find position of the sample point x_i
                int xiIdx = (int) (distance / step);
                float xi = xiIdx * step;
                float xi1 = xi + step;

                float tmpFpWeight = (distance - xi) / step * intpWeights[xiIdx+1] + (xi1 - distance) / step * intpWeights[xiIdx];

                // Get feature position of current frame, assuming match of features
                cv::Point2f currFtrPos = keypoints[bestFeatures[f]];
                
                // Weighted lookup vector
                lookupVector += tmpFpWeight * (currFtrPos - refFtrPos);

                // Accumulate temp feature point weight
                totalWeight += tmpFpWeight;
            }

            lookupVector *= 1.0 / totalWeight;
          
            // Interpolate pixel look up to smooth boundaries of morphed images
            // Mat::at<T>(y,x)
            float x = j + lookupVector.x;
            float y = i + lookupVector.y;

            m_alignedFrameData32f.at<cv::Vec3f>(i,j) = interpolatedPixelLookUp(x,y);
        }
    }
}


// Retrieve pixel (3-channels) at position (x,y)
// Boundary check performed
cv::Vec3f VideoFrame::getPixelAt(int x, int y)
{
    if (x >= 0 && x < m_frameData32f.size().width && y >= 0 && y < m_frameData32f.size().height)
    {
        // Return pixel at (x,y)
        return m_frameData32f.at<cv::Vec3f>(y,x);
    }
    else
    {
        // Out of bounds return black
        return cv::Vec3f(255.0, 255.0, 255.0);
    }
}


// Euclidean distance of vector
float VideoFrame::euclDist(cv::Point2f p)
{
    return std::sqrt(p.x * p.x + p.y * p.y);
}


// Linearly interpolate look up pixels
// Interpolate three neighbouring pixels
// @p relative look up vector 
// @x pixel index in x direction
// @y pixel index in y direction
cv::Vec3f VideoFrame::interpolatedPixelLookUp(float x, float y)
{
    // Absolute position of interpolation look up
    int ix = (int) x;
    int iy = (int) y;

    // Calculate fractional part
    float tailX = x - ix;
    float tailY = y - iy;

    // Interpolate with neighbour cells (right, lower, lower-right)
    cv::Vec3f resVec = (1.0 - tailY) * ((1.0 - tailX) * getPixelAt(ix,iy) + tailX * getPixelAt(ix+1,iy)) + tailY * ((1.0 - tailX) * getPixelAt(ix,iy+1) + tailX * getPixelAt(ix+1,iy+1));

    return resVec;                
}


// Weighting function (mirrored sigmoidal function)
float VideoFrame::weightFunction(float r)
{
    // Max distance in one dimension
    float rMax = std::max(m_frameData32f.size().width, m_frameData32f.size().height); 
     
    // Power distance function 
    return std::pow(0.9, (100 * r) / rMax) + 10 * std::exp(-0.1 / rMax * r * r) + 1; 
}


// Getter
// Get frame data
cv::Mat& VideoFrame::getFrameData()
{
    return m_frameData;
}

// Get frame data
cv::Mat& VideoFrame::getFrameData32f()
{
    return m_frameData32f;
}

// Get aligned frame data
cv::Mat& VideoFrame::getAlignedFrameData32f()
{
    return m_alignedFrameData32f;
}

// Get feature data
std::vector<FFeature>& VideoFrame::getFeatureData()
{
    return m_featureData;
}


// Get keypoints
std::vector<cv::Point2f>& VideoFrame::getKeypoints()
{
    return m_keypoints;
}


// Get status vec
std::vector<unsigned char>& VideoFrame::getStatusVec()
{
    return m_status;
}
