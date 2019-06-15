// Align two (consecutive) frames to stabilize video
// Using the feature based mapping method
void VideoFrame::alignFrameByFeatureBasedMorphing(const VideoFrame& referenceFrame, std::vector<int>& bestFeatures, int k)
{

    /* THIS IS FOR DEBUGGING PURPOSE ONLY
     * TODO REMOVE */
    cv::Mat imgIn;
    // THIS IS A HELPER IMAGE WITH SQUARES TO UNDERSTAND THE MOPRHING
    imgIn = cv::imread("img.jpg");
    cv::Mat imgOut(imgIn);
    /* TODO REMOVE END */

    // Iterate over all pixels in the frame (m_frameData)
    // In x and y direction
    for (int i = 0; i < m_frameData32f.size().height; ++i)
    {
        for (int j = 0; j < m_frameData32f.size().width; ++j)
        {
            // Initialize new weight and new lookup vector
            //@totalWeight: summed up weight over all features
            float totalWeight = 0.0;
            // LookupVector stores the final vector where to look up the new color
            cv::Point2f lookupVector = cv::Point2f(0.0,0.0);
            
            // Iterate over all feature and sum up weighted motion vector
            for (int f = 0; f < k; ++f)
            {
                // distance to features in referenceFrame (startFrame-1)
                //@distance: distance of pixel to specific feature
                cv::Point2f refFtrPos = referenceFrame.m_keypoints[bestFeatures[f]];
                float absX = (refFtrPos.x - j) * (refFtrPos.x - j);
                float absY = (refFtrPos.y - i) * (refFtrPos.y - i);
                float distance = std::sqrt(absX + absY);
               
                // Temporary feature point weight
                float tmpFpWeight = weightFunction(distance);
                
                // Get feature position of current frame, assuming match of features
                cv::Point2f currFtrPos = m_keypoints[bestFeatures[f]];
                
                // Weighted lookup vector
                lookupVector += tmpFpWeight * (currFtrPos - refFtrPos);

                // Accumulate temp feature point weight
                totalWeight += tmpFpWeight;
            }
            
            // Normalize lookupVector by weights
            lookupVector *= 1.0 / totalWeight;
          
            int v = (int) (j + lookupVector.x);
            int w = (int) (i + lookupVector.y);
            if (v >= 0 && v < m_frameData32f.size().width && w >= 0 && w < m_frameData32f.size().height)
            {
                // Linearly interpolate pixel look up 
                m_alignedFrameData.at<cv::Vec3f>(i,j) = interpolatedPixelLookUp(lookupVector, v, w);
                // DO THE SAME FOR THE DEBUGGING IMAGE
                imgOut.at<cv::Vec3b>(i,j) = imgIn.at<cv::Vec3b>(w,v);
            }
            else
            {
                // If lookup exceeds boundaries than fill up with black
                m_alignedFrameData.at<cv::Vec3f>(i,j) = cv::Vec3f(0.0,0.0,0.0);
                // TO THE SAME FOR THE DEBUGGING IMGAGE
                imgOut.at<cv::Vec3b>(i,j) = cv::Vec3b(0.0, 0.0, 0.0);
            }

        }
    }

    // STORE THE RESULT
    cv::imwrite("../images/raw/debugging.jpg", imgOut);
    drawMotionVec(m_frameData, referenceFrame, bestFeatures, k, false);
}

// Weighting function (mirrored sigmoidal function)
float VideoFrame::weightFunction(float r)
{
    // Max distance in one dimension
    float rMax = std::max(m_frameData32f.size().width, m_frameData32f.size().height); 
    // Power distance function
    return std::pow(0.9, (100 * r) / rMax) + 10 * std::exp(-0.1 / rMax * r * r) + 1; 
}

// Linearly interpolate look up pixels
// Interpolate three neighbouring pixels
// @p exact position of pixel to look up
// @v pixel index in x direction
// @w pixel index in y direction
cv::Vec3f VideoFrame::interpolatedPixelLookUp(cv::Point2f p, int v, int w)
{
    // Initialize resulting pixel color vector
    cv::Vec3f resVec = m_frameData32f.at<cv::Vec3f>(w,v);

    // Calculate fractional part
    float tailX = p.x - (int) p.x;
    float tailY = p.y - (int) p.y;

    // Interpolate
    if (false && (v + 1) < m_frameData32f.size().width && (w + 1) < m_frameData32f.size().height)
    {
        // Interpolate with the 3 neighbour cells (right, lower, right-lower)
        resVec = (1.0 - tailY) * ((1.0 - tailX) * m_frameData32f.at<cv::Vec3f>(w,v) + tailX * m_frameData32f.at<cv::Vec3f>(w+1,v)) + tailY * ((1.0 - tailX) * m_frameData32f.at<cv::Vec3f>(w,v+1) + tailX * m_frameData32f.at<cv::Vec3f>(w+1,v+1));
    }

    return resVec;                
}
