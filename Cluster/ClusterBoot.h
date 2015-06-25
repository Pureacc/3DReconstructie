#pragma once
#ifndef __CLUSTERBOOT_H_INCLUDED__
#define __CLUSTERBOOT_H_INCLUDED__

#include "stdafx.h"
/*
#include "ClusterRESTHandler.h"
#include <map>
using namespace web::http::experimental::listener;

template <class T>
class ClusterBoot {
	typedef pcl::PointCloud<T> PointCloud;
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
private:
	std::string coordinator_address;
	http_listener* listener;
	std::map<int, ClusterRESTHandler<T>*> clusters;
	boost::mutex clusters_mutex;
public:
	ClusterBoot(std::string _coordinator_address);
	void startListening();
};

template <class T>
ClusterBoot<T>::ClusterBoot(std::string _coordinator_address): coordinator_address(_coordinator_address) { }

template <class T>
void ClusterBoot<T>::startListening() {
}
*/

#include "ClusterRESTHandler.h"
#include <map>
using namespace web::http::experimental::listener;

template <class T>
class ClusterBoot {
	typedef pcl::PointCloud<T> PointCloud;
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
private:
	std::string coordinator_address;
	http_listener* listener;
	std::map<int, ClusterRESTHandler<T>*> clusters;
	boost::mutex clusters_mutex;
public:
	ClusterBoot(std::string _coordinator_address);
	void startListening();
	void viewModel(int cluster_id);
};

template <class T>
ClusterBoot<T>::ClusterBoot(std::string _coordinator_address): coordinator_address(_coordinator_address) { }

template <class T>
void ClusterBoot<T>::startListening() {
	listener = new http_listener(L"http://*:8080/create");
	listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "***Recieved creation POST.\n";
		//extract cluster ID from header
		int cluster_id;
		if (request.headers().has(U("Cluster_ID"))) {
			string_t cluster_idstr = request.headers().find(U("Cluster_ID"))->second;
			try{
				cluster_id = atoi(utility::conversions::to_utf8string(cluster_idstr).c_str());
			}
			catch(...) {
				std::cout << "ERROR: Couldn't parse cluster_id from header string.\n";
				request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't parse cluster_id from header string.")));
				return;
			}
			std::cout << "***Recieved cluster's ID: " << cluster_id << std::endl;
		}
		else {
			std::cout << "ERROR: Couldn't determine cluster ID.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine cluster ID.")));
			return;
		}
		//extract cloud from body
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "***Recieved pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		//write stream to file cloud_[cluster_id].pcd
		clusters_mutex.lock();
		map<int, ClusterRESTHandler<T>*>::iterator it = clusters.find(cluster_id);
		ClusterRESTHandler<T>* cluster;
		if (it == clusters.end()) {
			std::cout << "***Cluster ID " << cluster_id << " is a new cluster.\n";
			cluster = new ClusterRESTHandler<T>(cluster_id, coordinator_address);
			clusters.insert(pair<int,ClusterRESTHandler<T>*>(cluster_id,cluster));
			//cluster.startListeningClouds();
			//cluster.startHandlingClients();
		}
		else {
			std::cout << "WARNING: Cluster_id already exists, did not create new cluster" << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Cluster_id already exists.")));
			return;
		}
		clusters_mutex.unlock();
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (size != 0) {
			std::ofstream output;
			std::stringstream ss;
			ss << "cloud_" << cluster_id << ".pcd";
			std::string file_name = ss.str();
			output.open(file_name, std::ios::out|std::ios::binary);
			if (output.is_open()) {
				output.write(reinterpret_cast<char *>(text), size);
			}
			output.close();
			//read file cloud_[cluster_id].pcd to PointCloud
			if (pcl::io::loadPCDFile<T>(file_name, *cloud) == -1) //* load the file
			{
				std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
				request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
				return;
			}
			std::cout << "***Read PointCloud from " << file_name << std::endl;
			if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		}
		request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("Succesfully recieved create.")));
		cluster->setTargetModel(cloud);
		cluster->startListeningClouds();
		cluster->startHandlingClients();
		cluster->startListeningNDT();
		cluster->startListeningViewRequests();
		//boost::this_thread::sleep(boost::posix_time::seconds(30));
		//viewModel(0);
	});
	listener->open().wait();
	std::cout << "***Listening for cluster creations...\n";
}

template <class T>
void ClusterBoot<T>::viewModel(int cluster_id) {
	ClusterRESTHandler<T>* handler = clusters[cluster_id];
	handler->viewModel();
}


#endif