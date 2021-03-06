// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction and region based filtering
    
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vgrid;
    vgrid.setInputCloud(cloud);
    vgrid.setLeafSize(filterRes, filterRes, filterRes);
    vgrid.filter(*filteredCloud);

    // use cropbox to reduce the size of filtered cloud
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> croppedRegion(true);
    croppedRegion.setMin(minPoint);
    croppedRegion.setMax(maxPoint);
    croppedRegion.setInputCloud(filteredCloud);
    croppedRegion.filter(*cloudRegion);

    // remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true); // remove points
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
    // loop through the inliers and place in planeCloud
    for(int idx : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[idx]);
    }
    // create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

// TODO: RANSAC algorithm 
template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTolerance)
{
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    while(maxIterations--)
    {
        // randomly pick three points
        std::unordered_set<int> inliersSet;
        while(inliersSet.size() < 3)
            inliersSet.insert(rand() % (cloud->points.size()));

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliersSet.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        itr++;

        float a = ((y2-y1)*(z3-z1) - (z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
		float d = -(a*x1 + b*y1 + c*z1);

        for(int idx = 0; idx < cloud->points.size(); idx++)
        {
            if(inliersSet.count(idx) > 0)
                continue;

            pcl::PointXYZI point = cloud->points[idx];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // calculate distance from point to plane
            float dist = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

            if(dist <= distanceTolerance)
                inliersSet.insert(idx);
        }

        if(inliersSet.size() > inliersResult.size())
            inliersResult = inliersSet;
    }

    // change format from unordered set to point indices format
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(const auto& item : inliersResult)
    {
        inliers->indices.push_back(item);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliers;
}

// TODO: Custom segment plane algorithm using RANSAC 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // create new indice pointer
    pcl::PointIndices::Ptr inliers = Ransac3D(cloud, maxIterations, distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // create the segmentation object
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // set hyperparameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // random sample concensus
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

// custom clustering alogorithm using kdTree.h
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // create kdTree object
    KdTree* tree = new KdTree;
    std::vector<std::vector<int>> clusterIndices;
    std::vector<bool> isProcessed(cloud->points.size(), false);
    
    // insert points into kdTree
    for(int i=0; i < cloud->points.size(); i++)
        {
            std::vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            tree->insert(point, i);
        }

    int idx = 0;
    while(idx < cloud->points.size())
    {
        if(isProcessed[idx])
        {
            idx++;
            continue;
        }
        std::vector<int> cluster; // create new cluster
        // clusterProximity fx
        clusterProximity(idx, cloud, cluster, isProcessed, tree, clusterTolerance);
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
            clusterIndices.push_back(cluster);
        idx++;
    }

    for(auto& elem : clusterIndices)
    {
        // create new cloud cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int idx : elem)
            cloudCluster->points.push_back(cloud->points[idx]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;

}

// TODO: Clustering custom helper function
template<typename PointT>
void ProcessPointClouds<PointT>::clusterProximity(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &isProcessed, KdTree* tree, float clusterTolerance)
{
    isProcessed[idx] = true;
    cluster.push_back(idx);
    // check nearest points
    std::vector<float> point{cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
    std::vector<int> nearestPoints = tree->search(point, clusterTolerance);

    // check all nearest points to see if they are processed
    for(int id : nearestPoints)
    {
        // if not processed, run recursively
        if(!isProcessed[id])
            clusterProximity(id, cloud, cluster, isProcessed, tree, clusterTolerance);
    }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the  search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices>  clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices)
    {
        // create new cloud cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int idx : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[idx]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// template<typename PointT>
// BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
// {
//     // Find bounding box using PCA (principle component analysis)
//     /*
//         1. compute the centroid (c0, c1, c2) and the normalized covariance
//         2. compute the eigenvectors e0, e1, e2
//         3. move the points in that RF
//         4. compute the max, min and center of the diagonal
//         5. apply transformation to box centered at the origin
//             rotation = (e0, e1, e0Xe1)
//             translation = rotation * center_diag + (c0, c1, c2)
//     */
   
// }

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}