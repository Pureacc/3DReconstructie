
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/point_representation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cpprest\http_client.h>
#include <cpprest\http_listener.h>
#include <cpprest\producerconsumerstream.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include "FilterFunctions.h"


//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>
//#include <boost/thread/thread.hpp>
//#include <pcl/features/normal_3d.h>
//#include <pcl/registration/icp.h>

//#include "Client.h"