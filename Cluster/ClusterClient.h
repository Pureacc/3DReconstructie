#pragma once
//#ifndef __CLUSTERCLIENT_H_INCLUDED__
//#define __CLUSTERCLIENT_H_INCLUDED__

#include "stdafx.h"
//#include "ClusterRESTHandler.h"
//#include "ClusterRESTHandler.h"
//#include "FilterFunctions.h"
#include "CurvaturePointRepresentation.h"
/*
#include <pcl/registration/icp.h>
#include <cpprest\http_client.h>
#include <queue>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
*/

template <class T>
class ClusterRESTHandler;

template<class T>
class ClusterClient {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<T> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
private:
	Eigen::Matrix4f ndt_transformation;
	//ClusterRESTHandler<T> handler;
	ClusterRESTHandler<T>* handler;
	int client_id;
	std::queue<PointCloudPtr> frames;
	boost::mutex queue_mutex;
	boost::condition_variable queue_cond; //wait() notify_one()
	//boost::thread* thread; //Shouldn't be used. Only one client can merge with NDT at a time
	int frames_saved;
	bool recieved_all_frames;

	PointCloudPtr getNextFrame();
	//void registerLoop();
	typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal>::Ptr ClusterClient<T>::pairAlign(const PointCloudPtr cloud_src, const PointCloudPtr cloud_tgt, PointCloudPtr output, Eigen::Matrix4f &final_transform, float downsample_size);
public:
	ClusterClient(int client_id, ClusterRESTHandler<T>* _handler);
	void addFrame(PointCloudPtr cloud); //Assumes frame is already transformed with global transform at the point when ICP-model was sent
	int get_client_id() {return client_id;};
	int get_queue_size();
	int get_frames_saved() {return frames_saved;};
	void startRegistering();
	void recievedAllFrames() {recieved_all_frames = true;};
	void setNDTTransformation(Eigen::Matrix4f _ndt_transformation) {ndt_transformation = _ndt_transformation;};
};

template<class T>
ClusterClient<T>::ClusterClient(int _client_id, ClusterRESTHandler<T>* _handler): frames_saved(0), client_id(_client_id), recieved_all_frames(false), handler(_handler) {
	
}

template <class T>
typename pcl::PointCloud<T>::Ptr ClusterClient<T>::getNextFrame() {
	queue_mutex.lock();
	PointCloudPtr frame = frames.front();
	frames.pop();
	queue_mutex.unlock();
	return frame;
}

template <class T>
void ClusterClient<T>::startRegistering() {
	//start thread
	//thread = new boost::thread(&ClusterClient<T>::registerLoop, this);
	//get source frame
	std::cout << "Client " << client_id << " started registering.\n";
	while (get_queue_size() == 0) {
		boost::unique_lock<boost::mutex> lock(queue_mutex);
		queue_cond.wait(lock);
		lock.unlock();
	}
	PointCloudPtr source = getNextFrame();
	//std::cout << source->width * source->height << std::endl;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
	while (!recieved_all_frames) {
		while (get_queue_size() == 0) {
			boost::unique_lock<boost::mutex> lock(queue_mutex);
			queue_cond.wait(lock);
			lock.unlock();
		}
		PointCloudPtr target = getNextFrame();
		//Perform pairwise ICP and add to model
		PointCloudPtr temp (new PointCloud);
		PointCloudPtr result (new PointCloud);
		pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>::Ptr reg (new pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT>);
		//reg = pairAlign (source, target, temp, pairTransform, 0.025);
		reg = pairAlign (source, target, temp, pairTransform, 0.05);
		std::cout << "Client " << client_id << " performed ICP with fitness score " << reg->getFitnessScore() << std::endl;
		pcl::transformPointCloud (*temp, *result, GlobalTransform);
		PointCloudPtr end_result (new PointCloud);
		pcl::transformPointCloud (*result, *end_result, ndt_transformation);
		handler->addCloud(end_result);
		GlobalTransform = GlobalTransform * pairTransform;
		*source = *target;
	}
	//Add remaining frames
	while (get_queue_size() > 0) {
		PointCloudPtr target = getNextFrame();
		//Perform pairwise ICP and add to model
		PointCloudPtr temp (new PointCloud);
		PointCloudPtr result (new PointCloud);
		pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>::Ptr reg (new pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT>);
		//reg = pairAlign (source, target, temp, pairTransform, 0.025);
		reg = pairAlign (source, target, temp, pairTransform, 0.05);
		std::cout << "Client " << client_id << " performed ICP with fitness score " << reg->getFitnessScore() << std::endl;
		pcl::transformPointCloud (*temp, *result, GlobalTransform);
		PointCloudPtr end_result (new PointCloud);
		pcl::transformPointCloud (*result, *end_result, ndt_transformation);
		handler->addCloud(end_result);
		GlobalTransform = GlobalTransform * pairTransform;
		*source = *target;
	}
}

template<class T>
int ClusterClient<T>::get_queue_size() {
	queue_mutex.lock();
	int size = frames.size();
	queue_mutex.unlock();
	return size;
}

template<class T>
void ClusterClient<T>::addFrame(PointCloudPtr cloud) {
	queue_mutex.lock();
	frames.push(cloud);
	//queue_cond.notify_one();
	queue_mutex.unlock();
	queue_cond.notify_one();
	frames_saved++;
}

template <class T>
typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal>::Ptr ClusterClient<T>::pairAlign(const typename pcl::PointCloud<T>::Ptr cloud_src, const typename pcl::PointCloud<T>::Ptr cloud_tgt, typename pcl::PointCloud<T>::Ptr output, Eigen::Matrix4f &final_transform, float downsample_size)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  if (downsample_size != 0)
  {
		PCL_INFO ("Cloudsizes before ICP-downsampling: src(%d) tgt(%d) \n", cloud_src->height * cloud_src->width, cloud_tgt->height * cloud_tgt->width);
		src = filterfunctions::downSample<T>(cloud_src,downsample_size);
		tgt = filterfunctions::downSample<T>(cloud_tgt,downsample_size);
		PCL_INFO ("Cloudsizes after ICP-downsampling: src(%d) tgt(%d) \n", src->height * src->width, tgt->height * tgt->width);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
		PCL_INFO ("Cloudsizes ICP: src(%d) tgt(%d) \n", src->height * src->width, tgt->height * tgt->width);
  }

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<T, PointNormalT> norm_est;
  pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation ...
  CurvaturePointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>::Ptr reg (new pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT>);
  reg->setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  //reg.setMaxCorrespondenceDistance (0.1);
	reg->setMaxCorrespondenceDistance(0.5);
  // Set the point representation
  reg->setPointRepresentation (boost::make_shared<const CurvaturePointRepresentation> (point_representation));

  reg->setInputSource (points_with_normals_src);
  reg->setInputTarget (points_with_normals_tgt);

  //
  // Run the same optimization in a loop
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg->setMaximumIterations (2);
	
  for (int i = 0; i < 20; ++i)
  {
		points_with_normals_src = reg_result;
    // Estimate
    reg->setInputSource (points_with_normals_src);
    reg->align (*reg_result);
		//accumulate transformation between each Iteration
    Ti = reg->getFinalTransformation () * Ti;
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  final_transform = targetToSource;

	PCL_INFO("Has converged: (%d) Fitness score: (%f) Iterations: (%d) \n", reg->hasConverged(), reg->getFitnessScore(), reg->nr_iterations_);
	return reg;
}

//#endif