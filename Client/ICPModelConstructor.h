#ifndef __ICPModelConstructor_H_INCLUDED__
#define __ICPModelConstructor_H_INCLUDED__

#include "stdafx.h"
#include "FrameReader.h"

template <class T>
class ICPModelConstructor {
protected:
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<T> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
	
	typename pcl::PointCloud<T>::Ptr model;
	boost::mutex model_mutex;
	bool continueLoop;
	FrameReader<T> frameReader;
	Eigen::Matrix4f global_transform;

	typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal>::Ptr pairAlign (const typename pcl::PointCloud<T>::Ptr cloud_src, const typename pcl::PointCloud<T>::Ptr cloud_tgt, typename pcl::PointCloud<T>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);

public:
	ICPModelConstructor(char* fileName) : frameReader(fileName), model(new pcl::PointCloud<T>), global_transform(Eigen::Matrix4f::Identity ()) {}
	PointCloudPtr getResultantModel(float downSampleLeafSize = 0);
	void buildModel(float downSampleLeafSize); //continuously adds frames to model until stop() is called.
	PointCloudPtr getModel();
	FrameReader<T> getFrameReader(){return frameReader;};
	Eigen::Matrix4f getGlobalTransform(){return global_transform;};
	void stop(){continueLoop = false;};
};

template <class T>
typename pcl::PointCloud<T>::Ptr ICPModelConstructor<T>::getModel() {
	model_mutex.lock();
	pcl::PointCloud<T>::Ptr _model = model;
	model_mutex.unlock();
	return _model;
}

template <class T>
void ICPModelConstructor<T>::buildModel(float downSampleLeafSize) {
	std::cout << "Started building model.\n";
	frameReader.setIndex(0);
	PointCloud::Ptr source (new PointCloud);
	PointCloud::Ptr target (new PointCloud);
	continueLoop = true;
	if (frameReader.hasNextFrame()) {
		source = frameReader.getNextFrame();
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*source,*source,indices);
		source = filterfunctions::passThrough<PointT>(source,"z",1.0,3.0);
	}
	else
		continueLoop = false;

	if (frameReader.hasNextFrame()) {
		target = frameReader.getNextFrame();
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*target,*target,indices);
		target = filterfunctions::passThrough<PointT>(target,"z",1.0,3.0);
	}
	else
		continueLoop = false;

	Eigen::Matrix4f pairTransform;
	int addedClouds = 1;
	model_mutex.lock();
	*model = *source;
	model_mutex.unlock();
	std::cout << "Starting ICP loop.\n";
	while (continueLoop) {
		std::cout << "Loop iteration.\n";
		PointCloud::Ptr temp (new PointCloud);
		PointCloud::Ptr result (new PointCloud);
		pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT>::Ptr reg (new pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT>);
		reg = pairAlign (source, target, temp, pairTransform, true);
		//pcl::IterativeClosestPointNonLinear<T,T>::Ptr reg (new pcl::IterativeClosestPointNonLinear<T,T>);
		//reg = pairAlignWithoutNormals (source, target, temp, pairTransform, true);
		//pcl::GeneralizedIterativeClosestPoint<T,T> reg;
		//reg = pairAlignGeneralized(source, target, temp, pairTransform, true);
		pcl::transformPointCloud (*temp, *result, global_transform);
		model_mutex.lock();
		*model += *result;
		if (downSampleLeafSize != 0) {
			model = filterfunctions::downSample<PointT>(model,downSampleLeafSize);
		}
		model_mutex.unlock();
		addedClouds++;
		global_transform = global_transform * pairTransform;
		*source = *target;
		if (frameReader.hasNextFrame()) {
			target = frameReader.getNextFrame();
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*target, *target, indices);
			target = filterfunctions::passThrough<PointT>(target,"z",1.0,3.0);
		}
		else {
			continueLoop = false;
		}
	}
	frameReader.decrementIndex();
	PCL_INFO("Added (%d) clouds to ICP model.\n",addedClouds);
}

template <class T>
typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal>::Ptr ICPModelConstructor<T>::pairAlign(const typename pcl::PointCloud<T>::Ptr cloud_src, const typename pcl::PointCloud<T>::Ptr cloud_tgt, typename pcl::PointCloud<T>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  if (downsample)
  {
		PCL_INFO ("Cloudsizes before ICP-downsampling: src(%d) tgt(%d) \n", cloud_src->height * cloud_src->width, cloud_tgt->height * cloud_tgt->width);
		float leafSize = 0.025;
		src = filterfunctions::downSample<T>(cloud_src,leafSize);
		tgt = filterfunctions::downSample<T>(cloud_tgt,leafSize);
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
	//reg->setMaximumIterations(25);
	//reg->setInputSource(points_with_normals_src);
	//points_with_normals_src = reg_result;
	//reg->align(*reg_result);
	
  for (int i = 0; i < 20; ++i)
  {
		points_with_normals_src = reg_result;
    // Estimate
    reg->setInputSource (points_with_normals_src);
    reg->align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg->getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		/*
		//PCL_INFO("Difference between iterations transformation: (%f) \n", fabs ((reg.getLastIncrementalTransformation () - prev).sum ()));
    if (fabs ((reg->getLastIncrementalTransformation () - prev).sum ()) < reg->getTransformationEpsilon ())
			//reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.05);
			//PCL_INFO("IF \n");
      reg->setMaxCorrespondenceDistance (reg->getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg->getLastIncrementalTransformation ();
		*/
  }
	
	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();
	//targetToSource = reg->getFinalTransformation().inverse();
  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  //*output += *cloud_src;
  
  final_transform = targetToSource;

	PCL_INFO("Has converged: (%d) Fitness score: (%f) Iterations: (%d) \n", reg->hasConverged(), reg->getFitnessScore(), reg->nr_iterations_);
	return reg;
}

#endif