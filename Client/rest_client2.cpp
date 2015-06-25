#include "stdafx.h"
#include "ClientRESTHandler.h"

typedef pcl::PointXYZRGBA PointT;

int main(int argc, char* argv[])
{
	//char* server = "http://192.168.0.170:8080/register";
	//char* server = "http://192.168.1.2:8080/register";
	//char* server = "http://192.168.56.1:8080/register";
	char* server_ip = argv[2];
	//char* client_ip = "192.168.1.2"; //THUIS Put in argv?
	char* client_ip = argv[1];
	//char* server = "http://localhost:8080/cloud";
	//char* server = "http://78.22.26.250:8080/cloud";
	//char* server = "http://google.com";
	char* fileName = argv[3];
	ClientRESTHandler<PointT> client_handler(fileName,client_ip,server_ip);
	client_handler.startICPAndRegister();
	boost::this_thread::sleep_for(boost::chrono::seconds(30));
	//client_handler.viewGlobalModel();
	/*
	client_handler.registerToCoordinator("NewVidSecondRGB0.pcd");
	client_handler.postFrame("NewVidSecondRGB0.pcd");
	client_handler.postFrame("NewVidSecondRGB1.pcd");
	client_handler.postFrame("NewVidSecondRGB2.pcd");
	client_handler.postFrame("NewVidSecondRGB3.pcd");
	client_handler.postFrame("NewVidSecondRGB4.pcd");
	client_handler.postFrame("NewVidSecondRGB5.pcd");
	client_handler.postFrame("NewVidSecondRGB6.pcd");
	client_handler.postLastMessage();
	*/
	int a;
	std::cin >> a;
	return 0;
}
