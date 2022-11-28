#include "LinK3D_Extractor.h"


namespace BoW3D
{
    LinK3D_Extractor::LinK3D_Extractor(
            int nScans_, 
            float scanPeriod_, 
            float minimumRange_, 
            float distanceTh_,           
            int matchTh_):
            nScans(nScans_), 
            scanPeriod(scanPeriod_), 
            minimumRange(minimumRange_),   
            distanceTh(distanceTh_),          
            matchTh(matchTh_)
            {
                scanNumTh = ceil(nScans / 6);
                ptNumTh = ceil(1.5 * scanNumTh);                
            }

    void LinK3D_Extractor::removeClosedPointCloud(
            const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
            pcl::PointCloud<pcl::PointXYZ> &cloud_out)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x 
                + cloud_in.points[i].y * cloud_in.points[i].y 
                + cloud_in.points[i].z * cloud_in.points[i].z 
                < minimumRange * minimumRange)
            {
                continue;
            }
                
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }

        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    void LinK3D_Extractor::extractEdgePoint(
            pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, 
            ScanEdgePoints &edgePoints)
    {
        vector<int> scanStartInd(nScans, 0);
        vector<int> scanEndInd(nScans, 0);

        pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
        laserCloudIn = *pLaserCloudIn;
        vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
        removeClosedPointCloud(laserCloudIn, laserCloudIn);

        int cloudSize = laserCloudIn.points.size();
        float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;
    
        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }
        
        bool halfPassed = false;
        int count = cloudSize;
        pcl::PointXYZI point;
        vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(nScans);
        
        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (nScans == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (nScans - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (nScans == 64)
            {   
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = nScans / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
            }
            
            float ori = -atan2(point.y, point.x);
            if (!halfPassed)
            { 
                if (ori < startOri - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > startOri + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI)
                {
                    halfPassed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < endOri - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > endOri + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }

            point.intensity = ori;
            laserCloudScans[scanID].points.push_back(point);            
        }

        size_t scanSize = laserCloudScans.size();
        edgePoints.resize(scanSize);
        cloudSize = count;
                
        for(int i = 0; i < nScans; i++)
        {
            int laserCloudScansSize = laserCloudScans[i].size();
            if(laserCloudScansSize >= 15)
            {
                for(int j = 5; j < laserCloudScansSize - 5; j++)
                {
                    float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x
                                + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x
                                + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x
                                + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x
                                + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x
                                + laserCloudScans[i].points[j + 5].x;
                    float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y
                                + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y
                                + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y
                                + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y
                                + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y
                                + laserCloudScans[i].points[j + 5].y;
                    float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z
                                + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z
                                + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z
                                + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z
                                + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z
                                + laserCloudScans[i].points[j + 5].z;

                    float curv = diffX * diffX + diffY * diffY + diffZ * diffZ;
                    if(curv > 10 && curv < 20000)
                    {
                        float ori = laserCloudScans[i].points[j].intensity;
                        float relTime = (ori - startOri) / (endOri - startOri);

                        PointXYZSCA tmpPt;
                        tmpPt.x = laserCloudScans[i].points[j].x;
                        tmpPt.y = laserCloudScans[i].points[j].y;
                        tmpPt.z = laserCloudScans[i].points[j].z;
                        tmpPt.scan_position = i + scanPeriod * relTime;
                        tmpPt.curvature = curv;
                        tmpPt.angle = ori; 
                        edgePoints[i].emplace_back(tmpPt);
                    }
                }
            }
        }            
    }    

    //Roughly divide the areas to save time for clustering.
    void LinK3D_Extractor::divideArea(ScanEdgePoints &scanCloud, ScanEdgePoints &sectorAreaCloud)
    {
        sectorAreaCloud.resize(120); //The horizontal plane is divided into 120 sector area centered on LiDAR coordinate.
        int numScansPt = scanCloud.size();
        if(numScansPt == 0)
        {
            return;
        }
            
        for(int i = 0; i < numScansPt; i++) 
        {
            int numAScanPt = scanCloud[i].size();
            for(int j = 0; j < numAScanPt; j++)
            {                
                int areaID = 0;
                float angle = scanCloud[i][j].angle;
                
                if(angle > 0 && angle < 2 * M_PI)
                {
                    areaID = std::floor((angle / (2 * M_PI)) * 120);
                }   
                else if(angle > 2 * M_PI)
                {
                    areaID = std::floor(((angle - 2 * M_PI) / (2 * M_PI)) * 120);
                }
                else if(angle < 0)
                {
                    areaID = std::floor(((angle + 2 * M_PI) / (2 * M_PI)) * 120);
                }

                sectorAreaCloud[areaID].push_back(scanCloud[i][j]);
            }
        }
    }


    float LinK3D_Extractor::computeClusterMean(vector<PointXYZSCA> &cluster)
    {        
        float distSum = 0;
        int numPt = cluster.size();

        for(int i = 0; i < numPt; i++)
        {
            distSum += distXY(cluster[i]);
        }

        return (distSum/numPt);
    }

    void LinK3D_Extractor::computeXYMean(vector<PointXYZSCA> &cluster, std::pair<float, float> &xyMeans)
    {         
        int numPt = cluster.size();
        float xSum = 0;
        float ySum = 0;

        for(int i = 0; i < numPt; i++)
        {
            xSum += cluster[i].x;
            ySum += cluster[i].y;
        }

        float xMean = xSum/numPt;
        float yMean = ySum/numPt;
        xyMeans = std::make_pair(xMean, yMean);
    }

    void LinK3D_Extractor::getCluster(const ScanEdgePoints &sectorAreaCloud, ScanEdgePoints &clusters)
    {    
        ScanEdgePoints tmpclusters;
        PointXYZSCA curvPt;
        vector<PointXYZSCA> dummy(1, curvPt); 

        int numArea = sectorAreaCloud.size();

        //Cluster for each sector area.
        for(int i = 0; i < numArea; i++)
        {
            if(sectorAreaCloud[i].size() < 6)
                continue;

            int numPt = sectorAreaCloud[i].size();        
            ScanEdgePoints curAreaCluster(1, dummy);
            curAreaCluster[0][0] = sectorAreaCloud[i][0];

            for(int j = 1; j < numPt; j++)
            {
                int numCluster = curAreaCluster.size();

                for(int k = 0; k < numCluster; k++)
                {
                    float mean = computeClusterMean(curAreaCluster[k]);
                    std::pair<float, float> xyMean;
                    computeXYMean(curAreaCluster[k], xyMean);
                    
                    PointXYZSCA tmpPt = sectorAreaCloud[i][j];
                                        
                    if(abs(distXY(tmpPt) - mean) < distanceTh 
                        && abs(xyMean.first - tmpPt.x) < distanceTh 
                        && abs(xyMean.second - tmpPt.y) < distanceTh)
                    {
                        curAreaCluster[k].emplace_back(tmpPt);
                        break;
                    }
                    else if(abs(distXY(tmpPt) - mean) >= distanceTh && k == numCluster-1)
                    {
                        curAreaCluster.emplace_back(dummy);
                        curAreaCluster[numCluster][0] = tmpPt;
                    }
                    else
                    { 
                        continue; 
                    }                    
                }
            }

            int numCluster = curAreaCluster.size();
            for(int j = 0; j < numCluster; j++)
            {
                int numPt = curAreaCluster[j].size();

                if(numPt < ptNumTh)
                {
                    continue;
                }

                tmpclusters.emplace_back(curAreaCluster[j]);
            }
        }

        int numCluster = tmpclusters.size();
        
        vector<bool> toBeMerge(numCluster, false);
        multimap<int, int> mToBeMergeInd;
        set<int> sNeedMergeInd;

        //Merge the neighbor clusters.
        for(int i = 0; i < numCluster; i++)
        {
            if(toBeMerge[i]){
                continue;
            }
            float means1 = computeClusterMean(tmpclusters[i]);
            std::pair<float, float> xyMeans1;
            computeXYMean(tmpclusters[i], xyMeans1);

            for(int j = 1; j < numCluster; j++)
            {
                if(toBeMerge[j])
                {
                    continue;
                }

                float means2 = computeClusterMean(tmpclusters[j]);
                std::pair<float, float> xyMeans2;
                computeXYMean(tmpclusters[j], xyMeans2);

                if(abs(means1 - means2) < 2*distanceTh 
                    && abs(xyMeans1.first - xyMeans2.first) < 2*distanceTh 
                    && abs(xyMeans1.second - xyMeans2.second) < 2*distanceTh)
                {
                    mToBeMergeInd.insert(std::make_pair(i, j));
                    sNeedMergeInd.insert(i);
                    toBeMerge[i] = true;
                    toBeMerge[j] = true;
                }
            }

        }

        if(sNeedMergeInd.empty())
        {
            for(int i = 0; i < numCluster; i++)
            {
                clusters.emplace_back(tmpclusters[i]);
            }
        }
        else
        {
            for(int i = 0; i < numCluster; i++)
            {
                if(toBeMerge[i] == false)
                {
                    clusters.emplace_back(tmpclusters[i]);
                }
            }
            
            for(auto setIt = sNeedMergeInd.begin(); setIt != sNeedMergeInd.end(); ++setIt)
            {
                int needMergeInd = *setIt;
                auto entries = mToBeMergeInd.count(needMergeInd);
                auto iter = mToBeMergeInd.find(needMergeInd);
                vector<int> vInd;

                while(entries)
                {
                    int ind = iter->second;
                    vInd.emplace_back(ind);
                    ++iter;
                    --entries;
                }

                clusters.emplace_back(tmpclusters[needMergeInd]);
                size_t numCluster = clusters.size();

                for(size_t j = 0; j < vInd.size(); j++)
                {
                    for(size_t ptNum = 0; ptNum < tmpclusters[vInd[j]].size(); ptNum++)
                    {
                        clusters[numCluster - 1].emplace_back(tmpclusters[vInd[j]][ptNum]);
                    }
                }
            }
        }       
    }

    void LinK3D_Extractor::computeDirection(pcl::PointXYZI ptFrom, pcl::PointXYZI ptTo, Eigen::Vector2f &direction)
    {
        direction(0, 0) = ptTo.x - ptFrom.x;
        direction(1, 0) = ptTo.y - ptFrom.y;
    }

    vector<pcl::PointXYZI> LinK3D_Extractor::getMeanKeyPoint(const ScanEdgePoints &clusters, ScanEdgePoints &validCluster)
    {
        vector<pcl::PointXYZI> keyPoints;
        int numCluster = clusters.size();

        map<float, int> distanceOrder;

        int count = 0;
        vector<pcl::PointXYZI> tmpKeyPoints;
        ScanEdgePoints tmpEdgePoints;

        for(int i = 0; i < numCluster; i++)
        {
            int ptCnt = clusters[i].size();      
            if(ptCnt < ptNumTh)
            {
                continue;
            }

            vector<PointXYZSCA> tmpCluster;
            set<int> scans;
            float x = 0, y = 0, z = 0, intensity = 0;
            for(int ptNum = 0; ptNum < ptCnt; ptNum++)
            {
                PointXYZSCA pt = clusters[i][ptNum];          
                int scan = int(pt.scan_position);
                scans.insert(scan);

                x += pt.x;
                y += pt.y;
                z += pt.z;
                intensity += pt.scan_position;
            }

            if(scans.size() < (size_t)scanNumTh)
            {
                continue;
            }

            pcl::PointXYZI pt;
            pt.x = x/ptCnt;
            pt.y = y/ptCnt;
            pt.z = z/ptCnt;
            pt.intensity = intensity/ptCnt;

            float distance = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            auto iter = distanceOrder.find(distance);
            if(iter != distanceOrder.end())
            {
                continue;
            }

            distanceOrder[distance] = count; 

            count++;
            
            tmpKeyPoints.emplace_back(pt);
            tmpEdgePoints.emplace_back(clusters[i]);            
        }

        for(auto iter = distanceOrder.begin(); iter != distanceOrder.end(); iter++)
        {
            int index = (*iter).second;

            pcl::PointXYZI tmpPt = tmpKeyPoints[index];
            
            keyPoints.emplace_back(tmpPt);
            validCluster.emplace_back(tmpEdgePoints[index]);
        }
                
        return keyPoints;
    }

    float LinK3D_Extractor::fRound(float in)
    {
        float f;
        int temp = std::round(in * 10);
        f = temp/10.0;
        
        return f;
    }

    void LinK3D_Extractor::getDescriptors(const vector<pcl::PointXYZI> &keyPoints, 
                                          cv::Mat &descriptors)
    {
        if(keyPoints.empty())
        {
            return;
        }

        int ptSize = keyPoints.size();

        descriptors = cv::Mat::zeros(ptSize, 180, CV_32FC1); 

        vector<vector<float>> distanceTab;
        vector<float> oneRowDis(ptSize, 0);
        distanceTab.resize(ptSize, oneRowDis);

        vector<vector<Eigen::Vector2f>> directionTab;
        Eigen::Vector2f direct(0, 0);
        vector<Eigen::Vector2f> oneRowDirect(ptSize, direct);
        directionTab.resize(ptSize, oneRowDirect);

        //Build distance and direction tables for fast descriptor generation.
        for(size_t i = 0; i < keyPoints.size(); i++)
        {
            for(size_t j = i+1; j < keyPoints.size(); j++)
            {
                float dist = distPt2Pt(keyPoints[i], keyPoints[j]);
                distanceTab[i][j] = fRound(dist);
                distanceTab[j][i] = distanceTab[i][j];

                Eigen::Vector2f tmpDirection;
                                
                tmpDirection(0, 0) = keyPoints[j].x - keyPoints[i].x;
                tmpDirection(1, 0) = keyPoints[j].y - keyPoints[i].y;

                directionTab[i][j] = tmpDirection;
                directionTab[j][i] = -tmpDirection;
            }
        }

        for(size_t i = 0; i < keyPoints.size(); i++)
        {
            vector<float> tempRow(distanceTab[i]);
            std::sort(tempRow.begin(), tempRow.end());
            int Index[3];
           
            //Get the closest three keypoints of current keypoint.
            for(int k = 0; k < 3; k++)
            {                
                vector<float>::iterator it1 = find(distanceTab[i].begin(), distanceTab[i].end(), tempRow[k+1]); 
                if(it1 == distanceTab[i].end())
                {
                    continue;
                }
                else
                {
                    Index[k] = std::distance(distanceTab[i].begin(), it1);
                }
            }

            //Generate the descriptor for each closest keypoint. 
            //The final descriptor is based on the priority of the three closest keypoint.
            for(int indNum = 0; indNum < 3; indNum++)
            {
                int index = Index[indNum];
                Eigen::Vector2f mainDirection;
                mainDirection = directionTab[i][index];
                
                vector<vector<float>> areaDis(180);  
                areaDis[0].emplace_back(distanceTab[i][index]);
                          
                for(size_t j = 0; j < keyPoints.size(); j++)
                {
                    if(j == i || (int)j == index)
                    {
                        continue;
                    }
                    
                    Eigen::Vector2f otherDirection = directionTab[i][j];
                
                    Eigen::Matrix2f matrixDirect;
                    matrixDirect << mainDirection(0, 0), mainDirection(1, 0), otherDirection(0, 0), otherDirection(1, 0);
                    float deter = matrixDirect.determinant();

                    int areaNum = 0;
                    double cosAng = (double)mainDirection.dot(otherDirection) / (double)(mainDirection.norm() * otherDirection.norm());                                 
                    if(abs(cosAng) - 1 > 0)
                    {   
                        continue;
                    }
                                       
                    float angle = acos(cosAng) * 180 / M_PI;
                    
                    if(angle < 0 || angle > 180)
                    {
                        continue;
                    }
                    
                    if(deter > 0)
                    {
                        areaNum = ceil((angle - 1) / 2);                         
                    }
                    else
                    {
                        if(angle - 2 < 0)
                        { 
                            areaNum = 0;
                        }
                        else
                        {
                            angle = 360 - angle;
                            areaNum = ceil((angle - 1) / 2); 
                        }   
                    }

                    if(areaNum != 0)
                    {
                        areaDis[areaNum].emplace_back(distanceTab[i][j]);
                    }
                }
                
                float *descriptor = descriptors.ptr<float>(i);                                

                for(int areaNum = 0; areaNum < 180; areaNum++) 
                {
                    if(areaDis[areaNum].size() == 0)
                    {
                        continue;
                    }
                    else
                    {
                        std::sort(areaDis[areaNum].begin(), areaDis[areaNum].end());

                        if(descriptor[areaNum] == 0)
                        {
                            descriptor[areaNum] = areaDis[areaNum][0]; 
                        }                        
                    }
                }                
            }            
        }
    }

    void LinK3D_Extractor::match(
            vector<pcl::PointXYZI> &curAggregationKeyPt, 
            vector<pcl::PointXYZI> &toBeMatchedKeyPt,
            cv::Mat &curDescriptors, 
            cv::Mat &toBeMatchedDescriptors, 
            vector<pair<int, int>> &vMatchedIndex)
    {        
        int curKeypointNum = curAggregationKeyPt.size();
        int toBeMatchedKeyPtNum = toBeMatchedKeyPt.size();
        
        multimap<int, int> matchedIndexScore;      
        multimap<int, int> mMatchedIndex;
        set<int> sIndex;
       
        for(int i = 0; i < curKeypointNum; i++)
        {
            std::pair<int, int> highestIndexScore(0, 0);
            float* pDes1 = curDescriptors.ptr<float>(i);
            
            for(int j = 0; j < toBeMatchedKeyPtNum; j++)
            {
                int sameDimScore = 0;
                float* pDes2 = toBeMatchedDescriptors.ptr<float>(j); 
                
                for(int bitNum = 0; bitNum < 180; bitNum++)
                {                    
                    if(pDes1[bitNum] != 0 && pDes2[bitNum] != 0 && abs(pDes1[bitNum] - pDes2[bitNum]) <= 0.2){
                        sameDimScore += 1;
                    }
                    
                    if(bitNum > 90 && sameDimScore < 3){
                        break;                        
                    }                    
                }
               
                if(sameDimScore > highestIndexScore.second)
                {
                    highestIndexScore.first = j;
                    highestIndexScore.second = sameDimScore;
                }
            }
            
            //Used for removing the repeated matches.
            matchedIndexScore.insert(std::make_pair(i, highestIndexScore.second)); //Record i and its corresponding score.
            mMatchedIndex.insert(std::make_pair(highestIndexScore.first, i)); //Record the corresponding match between j and i.
            sIndex.insert(highestIndexScore.first); //Record the index that may be repeated matches.
        }

        //Remove the repeated matches.
        for(set<int>::iterator setIt = sIndex.begin(); setIt != sIndex.end(); ++setIt)
        {
            int indexJ = *setIt;
            auto entries = mMatchedIndex.count(indexJ);
            if(entries == 1)
            {
                auto iterI = mMatchedIndex.find(indexJ);
                auto iterScore = matchedIndexScore.find(iterI->second);
                if(iterScore->second >= matchTh)
                {                    
                    vMatchedIndex.emplace_back(std::make_pair(iterI->second, indexJ));
                }           
            }
            else
            { 
                auto iter1 = mMatchedIndex.find(indexJ);
                int highestScore = 0;
                int highestScoreIndex = -1;

                while(entries)
                {
                    int indexI = iter1->second;
                    auto iterScore = matchedIndexScore.find(indexI);
                    if(iterScore->second > highestScore){
                        highestScore = iterScore->second;
                        highestScoreIndex = indexI;
                    }                
                    ++iter1;
                    --entries;
                }

                if(highestScore >= matchTh)
                {                                       
                    vMatchedIndex.emplace_back(std::make_pair(highestScoreIndex, indexJ));                    
                }            
            }
        }
    }

    //Remove the edge keypoints with low curvature for further edge keypoints matching.
    void LinK3D_Extractor::filterLowCurv(ScanEdgePoints &clusters, ScanEdgePoints &filtered)
    {
        int numCluster = clusters.size();
        filtered.resize(numCluster);
        for(int i = 0; i < numCluster; i++)
        {
            int numPt = clusters[i].size();
            ScanEdgePoints tmpCluster;
            vector<int> vScanID;

            for(int j = 0; j < numPt; j++)
            {
                PointXYZSCA pt = clusters[i][j];
                int scan = int(pt.scan_position);
                auto it = std::find(vScanID.begin(), vScanID.end(), scan);

                if(it == vScanID.end())
                {
                    vScanID.emplace_back(scan);
                    vector<PointXYZSCA> vPt(1, pt);
                    tmpCluster.emplace_back(vPt);
                }
                else
                {
                    int filteredInd = std::distance(vScanID.begin(), it);
                    tmpCluster[filteredInd].emplace_back(pt);
                }
            }

            for(size_t scanID = 0; scanID < tmpCluster.size(); scanID++)
            {
                if(tmpCluster[scanID].size() == 1)
                {
                    filtered[i].emplace_back(tmpCluster[scanID][0]);
                }
                else
                {
                    float maxCurv = 0;
                    PointXYZSCA maxCurvPt;
                    for(size_t j = 0; j < tmpCluster[scanID].size(); j++)
                    {
                        if(tmpCluster[scanID][j].curvature > maxCurv)
                        {
                            maxCurv = tmpCluster[scanID][j].curvature;
                            maxCurvPt = tmpCluster[scanID][j];
                        }
                    }

                    filtered[i].emplace_back(maxCurvPt);
                }
            }  
        }
    }

    //Get the edge keypoint matches based on the matching results of aggregation keypoints.
    void LinK3D_Extractor::findEdgeKeypointMatch(
            ScanEdgePoints &filtered1, 
            ScanEdgePoints &filtered2, 
            vector<std::pair<int, int>> &vMatched, 
            vector<std::pair<PointXYZSCA, PointXYZSCA>> &matchPoints)
    {
        int numMatched = vMatched.size();
        for(int i = 0; i < numMatched; i++)
        {
            pair<int, int> matchedInd = vMatched[i];
                        
            int numPt1 = filtered1[matchedInd.first].size();
            int numPt2 = filtered2[matchedInd.second].size();

            map<int, int> mScanID_Index1;
            map<int, int> mScanID_Index2;

            for(int i = 0; i < numPt1; i++)
            {
                int scanID1 = int(filtered1[matchedInd.first][i].scan_position);
                pair<int, int> scanID_Ind(scanID1, i);
                mScanID_Index1.insert(scanID_Ind);
            }

            for(int i = 0; i < numPt2; i++)
            {
                int scanID2 = int(filtered2[matchedInd.second][i].scan_position);
                pair<int, int> scanID_Ind(scanID2, i);
                mScanID_Index2.insert(scanID_Ind);
            }

            for(auto it1 = mScanID_Index1.begin(); it1 != mScanID_Index1.end(); it1++)
            {
                int scanID1 = (*it1).first;
                auto it2 = mScanID_Index2.find(scanID1);
                if(it2 == mScanID_Index2.end()){
                    continue;
                }else{
                    vector<PointXYZSCA> tmpMatchPt;
                    PointXYZSCA pt1 = filtered1[matchedInd.first][(*it1).second];
                    PointXYZSCA pt2 = filtered2[matchedInd.second][(*it2).second];
                    
                    pair<PointXYZSCA, PointXYZSCA> matchPt(pt1, pt2);
                    matchPoints.emplace_back(matchPt);
                }
            }
        }
    }

    void LinK3D_Extractor::operator()(pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn, vector<pcl::PointXYZI> &keyPoints, cv::Mat &descriptors, ScanEdgePoints &validCluster)
    {
        ScanEdgePoints edgePoints;
        extractEdgePoint(pLaserCloudIn, edgePoints);

        ScanEdgePoints sectorAreaCloud;
        divideArea(edgePoints, sectorAreaCloud); 

        ScanEdgePoints clusters;
        getCluster(sectorAreaCloud, clusters); 
        
        vector<int> index;
        keyPoints = getMeanKeyPoint(clusters, validCluster);
          
        getDescriptors(keyPoints, descriptors); 
    }

}