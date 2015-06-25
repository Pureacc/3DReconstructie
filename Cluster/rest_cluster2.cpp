#include "stdafx.h"
//#include "ClusterBoot.h"
//#include "ClusterRESTHandler.h"
#include <iostream>
#include "ClusterBoot.h"
//#include "cpprest\http_listener.h"

//using namespace web::http::experimental::listener;
//using namespace concurrency::streams;
typedef pcl::PointXYZRGBA PointT;

int main(int argc, char* argv[])
{
	//ClusterRESTHandler<PointT> handler;
	//handler.startListeningClouds();
	//handler.startHandlingClients();
	//ClusterClient<PointT> client;
	std::string coordinator_address(argv[1]);
	ClusterBoot<PointT> boot(coordinator_address);
	boot.startListening();

	int a;
	std::cin >> a;
	return 0;
}
