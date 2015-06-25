#ifndef __FILTERFUNCTIONS_H_INCLUDED__
#define __FILTERFUNCTIONS_H_INCLUDED__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace filterfunctions {

	template<class T>
	typename pcl::PointCloud<T>::Ptr downSample(typename pcl::PointCloud<T>::Ptr cloud, float leafSize) {
		PCL_INFO("Cloud before downsampling: (%d) data points. \n", cloud->width * cloud->height);
		pcl::VoxelGrid<T> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(leafSize,leafSize,leafSize);
		pcl::PointCloud<T>::Ptr filteredCloud (new pcl::PointCloud<T>);
		sor.filter (*filteredCloud);
		PCL_INFO("Cloud after downsampling: (%d) data points. \n", filteredCloud->width * filteredCloud->height);
		return filteredCloud;
	};

	template<class T>
	typename pcl::PointCloud<T>::Ptr passThrough(typename pcl::PointCloud<T>::Ptr cloud, char* fieldName, float lowerLimit, float upperLimit) {
		PCL_INFO("Cloud before passthrough filtering: (%d) data points. \n", cloud->width * cloud->height);
		pcl::PointCloud<T>::Ptr filteredCloud (new pcl::PointCloud<T>);
		pcl::PassThrough<T> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName (fieldName);
		pass.setFilterLimits (lowerLimit, upperLimit);
		pass.filter (*filteredCloud);
		PCL_INFO("Cloud after passthrough filtering: (%d) data points. \n", filteredCloud->width * filteredCloud->height);
		return filteredCloud;
	};

	template<class T>
	typename pcl::PointCloud<T>::Ptr statisticalOutlierRemoval(typename pcl::PointCloud<T>::Ptr cloud, int meanK, double stddevMulThresh) {
		PCL_INFO("Cloud before statistical outlier removal: (%d) data points. \n", cloud->width * cloud->height);
		pcl::PointCloud<T>::Ptr filteredCloud (new pcl::PointCloud<T>);
		pcl::StatisticalOutlierRemoval<T> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (meanK);
		sor.setStddevMulThresh (stddevMulThresh);
		sor.filter (*filteredCloud);
		PCL_INFO("Cloud after statistical outlier removal: (%d) data points. \n", filteredCloud->width * filteredCloud->height);
		return filteredCloud;
	};

	template<class T>
	typename pcl::PointCloud<T>::Ptr radiusRemoval(typename pcl::PointCloud<T>::Ptr cloud, double radius, int neighbors) {
		PCL_INFO("Cloud before RadiusRemoval: (%d) data points. \n", cloud->width * cloud->height);
		pcl::PointCloud<T>::Ptr filteredCloud (new pcl::PointCloud<T>);
		pcl::RadiusOutlierRemoval<T> outrem;
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(radius);
		outrem.setMinNeighborsInRadius (neighbors);
		outrem.filter (*filteredCloud);
		PCL_INFO("Cloud after RadiusRemoval: (%d) data points. \n", filteredCloud->width * filteredCloud->height);
		return filteredCloud;
	};

}



#endif
