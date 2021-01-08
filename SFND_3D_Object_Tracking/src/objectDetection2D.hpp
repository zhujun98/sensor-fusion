
#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold, 
                   std::string basePath, std::string classesFile, std::string modelConfiguration, std::string modelWeights, bool bVis);

#endif /* objectDetection2D_hpp */
