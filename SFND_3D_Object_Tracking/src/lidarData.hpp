
#ifndef lidarData_hpp
#define lidarData_hpp

#include <stdio.h>
#include <fstream>
#include <string>

#include "dataStructures.h"

void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, float minX, float maxX, float maxY, float minZ, float maxZ, float minR);
void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, std::string filename);

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait=true);
void showLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidarPoints, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT, cv::Mat *extVisImg=nullptr);
#endif /* lidarData_hpp */
