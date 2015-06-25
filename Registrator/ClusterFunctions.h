#ifndef __CLUSTERFUNCTIONS_H_INCLUDED__
#define __CLUSTERFUNCTIONS_H_INCLUDED__

#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "FilterFunctions.h"
#include "kmeans.h"

namespace clusterfunctions {

	template <class T>
	void splitCloudHorizontally(const typename pcl::PointCloud<T>::Ptr inputCloud, typename pcl::PointCloud<T>::Ptr &outputFirst, typename pcl::PointCloud<T>::Ptr &outputSecond) {
		int size = inputCloud->size();
		std::vector<int> firstIndices(size/2);
		std::vector<int> secondIndices(size/2);
		for (int i=0; i<size/2; i++) {
			firstIndices[i] = i;
			secondIndices[i] = i+(size/2);
		}
		pcl::PointCloud<T>::Ptr temp1 (new pcl::PointCloud<T>(*inputCloud, firstIndices));
		outputFirst = temp1;
		pcl::PointCloud<T>::Ptr temp2 (new pcl::PointCloud<T>(*inputCloud, secondIndices));
		outputSecond = temp2;
	}

	template <class T>
	void splitCloudVertically(const typename pcl::PointCloud<T>::Ptr inputCloud, typename pcl::PointCloud<T>::Ptr &outputFirst, typename pcl::PointCloud<T>::Ptr &outputSecond) {
		int size = inputCloud->size();
		std::vector<int> firstIndices(size/2);
		std::vector<int> secondIndices(size/2);
		int index = 0;
		int height = inputCloud->height;
		int width = inputCloud->width;
		for (int i=0; i<height; i++) {
			for (int j=0; j<width/2; j++) {
				firstIndices[index] = i*width + j;
				secondIndices[index] = i*width + j + width/2;
				index++;
			}
		}
		pcl::PointCloud<T>::Ptr temp1 (new pcl::PointCloud<T>(*inputCloud, firstIndices));
		outputFirst = temp1;
		pcl::PointCloud<T>::Ptr temp2 (new pcl::PointCloud<T>(*inputCloud, secondIndices));
		outputSecond = temp2;
	}

	template <class T>
	void euclideanClustering(const typename pcl::PointCloud<T>::Ptr inputCloud, std::vector<typename pcl::PointCloud<T>::Ptr> &clouds) {
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::PointCloud<T>::Ptr cloud_f (new pcl::PointCloud<T>);
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::PointCloud<T>::Ptr cloud_filtered (new pcl::PointCloud<T>);
		cloud_filtered = filterfunctions::downSample<T>(inputCloud,0.01);

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<T> seg;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointCloud<T>::Ptr cloud_plane (new pcl::PointCloud<T> ());
		pcl::PCDWriter writer;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.02);

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<T> extract;
			extract.setInputCloud (cloud_filtered);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);
			std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud_filtered = *cloud_f;
		}

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
		tree->setInputCloud (cloud_filtered);

		pcl::EuclideanClusterExtraction<T> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<T>::Ptr cloud_cluster (new pcl::PointCloud<T>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			clouds.push_back(cloud_cluster);
			j++;
		}
		std::cout << "Number of clusters: " << j;
	}

	template <class T>
	void octreeSplit(const typename pcl::PointCloud<T>::Ptr inputCloud, typename pcl::PointCloud<T>::Ptr &outputFirst, typename pcl::PointCloud<T>::Ptr &outputSecond) {
		int points = inputCloud->width * inputCloud->height;
		float resolution = 1000.0f;
		pcl::octree::OctreePointCloudPointVector<T>::Ptr tree(new pcl::octree::OctreePointCloudPointVector<T>(resolution));
		tree->setInputCloud(inputCloud);
		tree->defineBoundingBox();
		tree->addPointsFromInputCloud();
		//PCL_INFO("tree resolution: %f with %d data points and %d levels deep \n", tree.getResolution(), tree.getIndices()->size(), tree.getTreeDepth());
		PCL_INFO("Tree resolution: %f maximum depth: %d \n", tree->getResolution(), tree->getTreeDepth());
    pcl::octree::OctreePointCloudPointVector<T>::LeafNodeIterator it;
    const pcl::octree::OctreePointCloudPointVector<T>::LeafNodeIterator it_end = tree->leaf_end();
    unsigned int leafNodeCounter = 0;
		std::vector<int> indicesFirst;
		std::vector<int> indicesSecond;
		int added = 0;
		int totalLeafs = 0;
		for (it = tree->leaf_begin(); it != it_end; ++it)
			totalLeafs++;
		it.reset();
    for (it = tree->leaf_begin(); it != it_end; ++it)
    {
			pcl::octree::OctreeNode* leafNode = it.getCurrentOctreeNode();
			pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
			std::vector<int> indexVector;
      container.getPointIndices(indexVector);
			//if (added < points/2) {
			if (leafNodeCounter<totalLeafs/2) {
				//Add to first
				for (int i=0; i<indexVector.size(); i++) {
					indicesFirst.push_back(indexVector[i]);
					added++;
				}
			}
			else {
				//Add to second
				for (int i=0; i<indexVector.size(); i++) {
					indicesSecond.push_back(indexVector[i]);
					added++;
				}
			}
			
      leafNodeCounter++;
    }
		/*
		std::sort(indicesFirst.begin(),indicesFirst.end());
		std::sort(indicesSecond.begin(),indicesSecond.end());
		PCL_INFO("Iterated %d leafnodes.\n", leafNodeCounter);
		PCL_INFO("First cloud contains %d points.\n", outputFirst->width * outputFirst->height);
		PCL_INFO("Second cloud contains %d points.\n", outputSecond->width * outputSecond->height);
		PCL_INFO("First indices: \n");
		for (int i=0; i<500; i++) {
			cout << indicesFirst[i] << " ";
		}
		PCL_INFO("\n \n");
		PCL_INFO("Second indices: \n");
		for (int i=0; i<500; i++) {
			cout << indicesSecond[i] << " ";
		}
		*/
		
		pcl::PointCloud<T>::Ptr temp1 (new pcl::PointCloud<T>(*inputCloud, indicesFirst));
		outputFirst = temp1;
		pcl::PointCloud<T>::Ptr temp2 (new pcl::PointCloud<T>(*inputCloud, indicesSecond));
		outputSecond = temp2;
		PCL_INFO("Iterated %d leafnodes.\n", leafNodeCounter);
		PCL_INFO("Input cloud contains %d points.\n", inputCloud->width * inputCloud->height);
		PCL_INFO("First cloud contains %d points.\n", outputFirst->width * outputFirst->height);
		PCL_INFO("Second cloud contains %d points.\n", outputSecond->width * outputSecond->height);
	}

	template <class T>
	void kMeansSplit(const typename pcl::PointCloud<T>::Ptr inputCloud, typename pcl::PointCloud<T>::Ptr outputFirst, typename pcl::PointCloud<T>::Ptr outputSecond) {
		
		std::vector<T,Eigen::aligned_allocator<T> > pointsCloud = inputCloud->points;
		pcl::Kmeans kMeans(pointsCloud.size(), 3);
		std::vector<std::vector<float> > points(pointsCloud.size());
		for (int i=0; i<pointsCloud.size(); i++) {
			std::vector<float> point;
			point.push_back(pointsCloud[i].x);
			point.push_back(pointsCloud[i].y);
			point.push_back(pointsCloud[i].z);
			points[i] = point;
		}
		PCL_INFO("pointsvector contains %d points. \n",points.size());
		kMeans.setInputData(points);
		kMeans.setClusterSize(2);
		kMeans.kMeans();
		std::vector<int> pointsToClusters = kMeans.getPointsToClusters();
		for (int i=0; i<pointsCloud.size(); i++) {
			if (pointsToClusters[i] == 0) {
				outputFirst->push_back(pointsCloud[i]);
			}
			else {
				outputSecond->push_back(pointsCloud[i]);
			}
		}
		PCL_INFO("Completed Kmeans in %d iterations. \n", kMeans.get_iterations());
		PCL_INFO("First cloud contains %d points.\n", outputFirst->width * outputFirst->height);
		PCL_INFO("Second cloud contains %d points.\n", outputSecond->width * outputSecond->height);
	}

}


#endif
