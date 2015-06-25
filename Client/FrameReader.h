#ifndef __FRAMEREADER_H_INCLUDED__
#define __FRAMEREADER_H_INCLUDED__

#include "stdafx.h"
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <boost/make_shared.hpp>
//#include <pcl/io/pcd_io.h>

template <class T>
class FrameReader {
typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
typedef pcl::PointCloud<T> PointCloud;
private:
	char* fileName;
	int index; //points to the index of the frame 1 unit further than the one last returned
	int skipFrames;

public:
	FrameReader(char* fileName_);
	FrameReader(char* fileName_, int index_, int skipFrames_);
	bool hasNextFrame();
	typename pcl::PointCloud<T>::Ptr getNextFrame();
	std::string getNextFrameName();
	void decrementIndex() {index--;};
	void setIndex(int index_) {index = index_;}
	int getIndex() {return index;}
};

template <class T>
FrameReader<T>::FrameReader(char* fileName_) : fileName(fileName_) , index(0), skipFrames(0) {}

template <class T>
FrameReader<T>::FrameReader(char* fileName_,  int index_, int skipFrames_) : fileName(fileName_) , index(index_), skipFrames(skipFrames_) {}

template <class T>
bool FrameReader<T>::hasNextFrame() {
	PointCloudPtr cloud (new PointCloud);
	std::stringstream ss;
	ss << fileName << index << ".pcd";
	std::string file = ss.str();
	if (pcl::io::loadPCDFile<T> (file, *cloud) == -1)
  {
    return false;
  }
	else
		return true;
}

template <class T>
typename pcl::PointCloud<T>::Ptr FrameReader<T>::getNextFrame() {
	PointCloudPtr cloud (new PointCloud);
	std::stringstream ss;
	ss << fileName << index << ".pcd";
	std::string file = ss.str();
	if (pcl::io::loadPCDFile<T> (file, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read frame %s \n",file);
  }
	index = index + 1 + skipFrames;
	return cloud;
}

template <class T>
std::string FrameReader<T>::getNextFrameName() {
	std::stringstream ss;
	ss << fileName << index << ".pcd";
	index = index + 1 + skipFrames;
	return ss.str();
}

#endif