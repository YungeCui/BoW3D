#pragma once

#include <vector>
#include <map>
#include <set>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;


struct PointXYZSCA
{
    PCL_ADD_POINT4D;
    float scan_position;
    float curvature;
    float angle;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZSCA, 
    (float, x, x)(float, y, y)(float, z, z)(float, scan_position, scan_position)(float, curvature, curvature)(float, angle, angle))

typedef vector<vector<PointXYZSCA>> ScanEdgePoints;

namespace BoW3D
{
    #define EdgePointCloud pcl::PointCloud<PointXYZSCA>
    #define distXY(a) sqrt(a.x * a.x + a.y * a.y)
    #define distOri2Pt(a) sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    #define distPt2Pt(a, b) sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z))
    
    using std::atan2;
    using std::cos;
    using std::sin;
   
    class Frame;

    class LinK3D_Extractor
    {
        public:
            LinK3D_Extractor(int nScans_, float scanPeriod_, float minimumRange_, float distanceTh_, int matchTh_);

            ~LinK3D_Extractor(){}

            bool comp (int i, int j) 
            { 
                return cloudCurvature[i] < cloudCurvature[j]; 
            }

            void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                        pcl::PointCloud<pcl::PointXYZ> &cloud_out);      

            void extractEdgePoint(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, ScanEdgePoints &edgePoints);

            void divideArea(ScanEdgePoints &scanEdgePoints, ScanEdgePoints &sectorAreaCloud);

            float computeClusterMean(vector<PointXYZSCA> &cluster);

            void computeXYMean(vector<PointXYZSCA> &cluster, pair<float, float> &xyMeans);

            void getCluster(const ScanEdgePoints &sectorAreaCloud, ScanEdgePoints &clusters);

            void computeDirection(pcl::PointXYZI ptFrom, 
                                  pcl::PointXYZI ptTo, 
                                  Eigen::Vector2f &direction);

            vector<pcl::PointXYZI> getMeanKeyPoint(const ScanEdgePoints &clusters, 
                                                   ScanEdgePoints &validCluster);
            
            float fRound(float in);
                        
            void getDescriptors(const vector<pcl::PointXYZI> &keyPoints, cv::Mat &descriptors);
            
            void match(vector<pcl::PointXYZI> &curAggregationKeyPt, 
                       vector<pcl::PointXYZI> &toBeMatchedKeyPt,
                       cv::Mat &curDescriptors, 
                       cv::Mat &toBeMatchedDescriptors, 
                       vector<pair<int, int>> &vMatchedIndex);

            void filterLowCurv(ScanEdgePoints &clusters, ScanEdgePoints &filtered);

            void findEdgeKeypointMatch(ScanEdgePoints &filtered1, 
                                       ScanEdgePoints &filtered2, 
                                       vector<pair<int, int>> &vMatched, 
                                       vector<pair<PointXYZSCA, PointXYZSCA>> &matchPoints);
            
            void operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, 
                            vector<pcl::PointXYZI> &keyPoints, 
                            cv::Mat &descriptors, 
                            ScanEdgePoints &validCluster);

        private:
            int nScans;
            float scanPeriod;
            float minimumRange;

            float distanceTh;
            int matchTh;           
            int scanNumTh;
            int ptNumTh;

            float cloudCurvature[400000];
    };
}
