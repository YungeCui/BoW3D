#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "Frame.h"
#include "LinK3D_Extractor.h"


using namespace std;

namespace BoW3D
{
    class Frame;
    class LinK3D_extractor;
    
    template <typename T>
    inline void hash_combine(std::size_t &seed, const T &val) 
    {
        seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    
    template <typename T> 
    inline void hash_val(std::size_t &seed, const T &val) 
    {
        hash_combine(seed, val);
    }

    template <typename T1, typename T2>
    inline void hash_val(std::size_t &seed, const T1 &val1, const T2 &val2) 
    {
        hash_combine(seed, val1);
        hash_val(seed, val2);
    }

    template <typename T1, typename T2>
    inline std::size_t hash_val(const T1 &val1, const T2 &val2) 
    {
        std::size_t seed = 0;
        hash_val(seed, val1, val2);
        return seed;
    }

    struct pair_hash 
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2> &p) const {
            return hash_val(p.first, p.second);
        }
    };
   
    class BoW3D: public unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>  //位值 位ID, 帧ID 描述子ID
    {
        public:
            BoW3D(LinK3D_Extractor* pLinK3D_Extractor, float thr_, int thf_, int num_add_retrieve_features_);

            ~BoW3D(){}
            
            void update(Frame* pCurrentFrame);

            int loopCorrection(Frame* currentFrame, Frame* matchedFrame, vector<pair<int, int>> &vMatchedIndex, Eigen::Matrix3d &R, Eigen::Vector3d &t);

            void retrieve(Frame* pCurrentFrame, int &loopFrameId, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt);           

        private:
            LinK3D_Extractor* mpLinK3D_Extractor;

            std::pair<int, int> N_nw_ofRatio; //Used to compute the ratio in our paper.          

            vector<Frame*> mvFrames;

            float thr; //Ratio threshold in our paper.
            int thf; //Frequency threshold in our paper.
            int num_add_retrieve_features; //The number of added or retrieved features for each frame.
    };
}
