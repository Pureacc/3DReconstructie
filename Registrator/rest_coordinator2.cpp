#include "stdafx.h"
#include "CoordinatorRESTHandler.h"
//#include "cpprest\http_listener.h"

using namespace web::http::experimental::listener;
using namespace concurrency::streams;
typedef pcl::PointXYZRGBA PointT;

int main(int argc, char* argv[])
{
	CoordinatorRESTHandler<PointT> coordinator_handler;
	for(int i=1; i<argc; i++) {
		coordinator_handler.addAvailableIP(std::string(argv[i]));
	}
	coordinator_handler.startListeningClients();
	coordinator_handler.startListeningSplits();
	coordinator_handler.startListeningClientUpdates();
	int a;
	std::cin >> a;
	return 0;
}
