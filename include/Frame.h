#pragma once

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LinK3D_Extractor.h"


using namespace std;
using namespace Eigen;

namespace BoW3D
{
    class LinK3D_Extractor;

    class Frame
    {
        public:
            Frame();
            
            Frame(LinK3D_Extractor* pLink3dExtractor, pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn);
            
            ~Frame(){};
                        
        public: 
            static long unsigned int nNextId;

            long unsigned int mnId;
            
            LinK3D_Extractor* mpLink3dExtractor;

            ScanEdgePoints mClusterEdgeKeypoints;

            std::vector<pcl::PointXYZI> mvAggregationKeypoints;
    
            cv::Mat mDescriptors;                
    };

}

