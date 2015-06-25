#pragma once
//#ifndef __CLUSTERRESTHANDLER_H_INCLUDED__
//#define __CLUSTERRESTHANDLER_H_INCLUDED__

#include "stdafx.h"
#include <map>
#include <queue>
#include <stdlib.h>
#include "ClusterClient.h"
#include "ConversionsExtra.h"
//#include <boost/make_shared.hpp>
//#include <boost/thread/thread.hpp>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/ndt.h>


using namespace std;
using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace web::http::experimental::listener;

//class ClusterClient;

template <class T>
class ClusterRESTHandler {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<T> PointCloud;
private:
	PointCloudPtr targetModel;
	static const int MAX_MODEL_POINTS = 1000000;
	boost::mutex model_mutex;
	http_listener* clouds_listener;
	http_listener* ndt_listener;
	map<int, ClusterClient<T>*> clients;
	queue<ClusterClient<T>*> clients_queue;
	boost::thread* queue_handler;
	boost::mutex clients_mutex;
	boost::mutex clients_queue_mutex;
	boost::condition_variable queue_condition;
	bool handling;
	int cluster_id;
	std::string coordinator_address;

	ClusterClient<T>* getNextClient();
	int clients_queue_size();
	void clientHandlerLoop();
	double testNDT(const PointCloudPtr inputCloud, float downSampleLeafSize);
	void save_to_file(typename pcl::PointCloud<T>::Ptr cloud, const char* file_name);
	void checkModelSplit();
public:
	ClusterRESTHandler(int _cluster_id, std::string _coordinator_address);
	void startListeningClouds();
	void startListeningNDT();
	void startListeningViewRequests();
	void startHandlingClients();
	void listenSplit(); //Blocks until message recieved or exception
	void stopHandlingClients();
	void setTargetModel(typename pcl::PointCloud<T>::Ptr model) {*targetModel = *model;};
	typename pcl::NormalDistributionsTransform<T,T>::Ptr registerModel(const PointCloudPtr inputCloud, float downSampleLeafSize);
	void addCloud(PointCloudPtr inputCloud); //Assumes inputcloud is already correctly transformed to targetModel
	void visualizeCloud(PointCloudPtr cloud);
	void viewModel();
};

template <class T>
ClusterRESTHandler<T>::ClusterRESTHandler(int _cluster_id, std::string _coordinator_address): targetModel(new PointCloud()), cluster_id(_cluster_id), coordinator_address(_coordinator_address) {
	
}

template <class T>
ClusterClient<T>* ClusterRESTHandler<T>::getNextClient() {
	clients_queue_mutex.lock();
	std::cout << "ENTERED CLIENTS QUEUE LOCK.\n";
	ClusterClient<T>* client = clients_queue.front();
	std::cout << "GOT NEXT CLIENT ID " << client->get_client_id() << std::endl;
	clients_queue.pop();
	clients_queue_mutex.unlock();
	return client;
}

template <class T>
void ClusterRESTHandler<T>::listenSplit() {
	std::stringstream ss;
	ss << "http://*:8080/" << cluster_id << "/split";
	const wchar_t* wc_server_address = conversionsExtra::toWchar(ss.str().c_str());
	http_listener split_listener(wc_server_address);
	delete wc_server_address;
	bool recieved_answer = false;
	split_listener.support(web::http::methods::POST, [this, &recieved_answer](web::http::http_request request) {
		std::cout << "***Recieved split POST.\n";
		//extract cloud from body
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "***Recieved pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		//write stream to file cloud_[value].pcd
		std::ofstream output;
		std::string file_name = std::string("model_split.pcd");
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			output.write(reinterpret_cast<char *>(text), size);
		}
		output.close();
		//read file cloud_[client_id]_[framenumber].pcd to PointCloud
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: Split model reciever couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Cluster couldn't read pointcloud file")));
			return;
		}
		request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("Succesfully recieved split model.")));
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		model_mutex.lock();
		*targetModel = *cloud;
		model_mutex.unlock();
		recieved_answer = true;
	});
	split_listener.open().wait();
	std::cout << "***Listening for split model answer from coordinator...\n";
	while (!recieved_answer) {
		boost::this_thread::sleep( boost::posix_time::seconds(1) );
	}
	split_listener.close();
	model_mutex.lock();
	std::cout << "Split completed. New model contains " << targetModel->width * targetModel->height << " data points.\n";
	model_mutex.unlock();
}

template <class T>
void ClusterRESTHandler<T>::checkModelSplit() {
	std::cout << "Checking if model should be split...\n";
	model_mutex.lock();
	int size = targetModel->height * targetModel->width;
	model_mutex.unlock();
	if (size > MAX_MODEL_POINTS) {
		//Save model to file
		const char* file_name = "model_split.pcd";
		model_mutex.lock();
		save_to_file(targetModel, file_name);
		model_mutex.unlock();
		//Send request to coordinator for split with model in body and cluster_id in header (ID)
		std::stringstream uriss;
		uriss << "http://" << coordinator_address << ":8080/split";
		std::cout << "Requesting split to coordinator on address " << uriss.str() << std::endl;
		http_client client(utility::conversions::to_string_t(uriss.str()));
		std::streampos size;
		char * memblock;
		std::ifstream file (file_name, std::ios::in|std::ios::binary|std::ios::ate);
		if (file.is_open())
		{
			size = file.tellg();
			memblock = new char [size];
			file.seekg (0, std::ios::beg);
			file.read (memblock, size);
			file.close();
			std::cout << "Read file " << file_name << ".\n";
		} 
		else std::cout << "ERROR: Unable to open file " << file_name;
		size_t bufferSize = size;
		// Put text to the buffer
		concurrency::streams::producer_consumer_buffer<char> buffer;
		buffer.putn((const char*)memblock, bufferSize).wait();
		buffer.close(std::ios_base::out);
		if(remove(file_name) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		// Create HTTP request
		http_request msg;
		msg.set_body(concurrency::streams::istream(buffer));
		msg.set_method(methods::POST);
		msg.headers().set_content_length(size);
		msg.headers().add<int>(U("ID"),cluster_id);
		std::cout << "Built request.\n";
		int statusCode;
		// Send HTTP request
		auto requestTask = client.request(msg).then([&statusCode, &memblock](http_response response) {
			statusCode = response.status_code();
			delete memblock;
		});
		try {
			requestTask.wait();
		}
		catch (const std::exception &e) {
			printf("Error exception:%s\n", e.what());
			delete memblock;
			return;
		}
		std::cout << "Request sent.\n";
		//Listen until new model recieved
		listenSplit();
	}
	else {
		std::cout << "Model not big enough to split yet.\n";
	}
}

template <class T>
int ClusterRESTHandler<T>::clients_queue_size() {
	clients_queue_mutex.lock();
	int size = clients_queue.size();
	clients_queue_mutex.unlock();
	return size;
}

template <class T>
void ClusterRESTHandler<T>::startHandlingClients() {
	handling = true;
	queue_handler = new boost::thread(&ClusterRESTHandler<T>::clientHandlerLoop, this);
}

template <class T>
void ClusterRESTHandler<T>::stopHandlingClients() {
	handling = false;
}

template <class T>
void ClusterRESTHandler<T>::clientHandlerLoop() {
	std::cout << "Started handling clients...\n";
	int size = clients_queue_size();
	while (handling) {
		if (size == 0) {
			boost::unique_lock<boost::mutex> lock(clients_queue_mutex);
			queue_condition.wait(lock);
			lock.unlock();
		}
		ClusterClient<T>* client = getNextClient();
		//start client's ICP+NDT
		client->startRegistering();
		std::cout << "Finished registering frames for client " << client->get_client_id() << std::endl;
		std::stringstream ss;
		ss << "Model_" << cluster_id << ".pcd";
		model_mutex.lock();
		save_to_file(targetModel, ss.str().c_str());
		model_mutex.unlock();
		checkModelSplit();
		size = clients_queue_size();
	}
}

template <class T>
void ClusterRESTHandler<T>::startListeningNDT() {
	std::stringstream ss;
	ss << "http://*:8080/" << cluster_id << "/ndt";
	const wchar_t* wc_server_address = conversionsExtra::toWchar(ss.str().c_str());
	ndt_listener = new http_listener(wc_server_address);
	delete wc_server_address;
	ndt_listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "***Recieved ndt POST.\n";
		//extract value from header
		string_t value;
		if (request.headers().has(U("value"))) {
			value = request.headers().find(U("value"))->second;
			std::cout << "***Recieved ndt request value\n";
		}
		else {
			std::cout << "ERROR: Couldn't determine ndt request value.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine ndt request value.")));
			return;
		}
		//extract cloud from body
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "***Recieved client pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		//write stream to file cloud_[value].pcd
		std::ofstream output;
		std::stringstream ss;
		ss << "cloud_" << utility::conversions::to_utf8string(value) << ".pcd";
		std::string file_name = ss.str();
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			output.write(reinterpret_cast<char *>(text), size);
		}
		output.close();
		//read file cloud_[client_id]_[framenumber].pcd to PointCloud
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: NDT request couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
			return;
		}
		request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("Succesfully recieved ndt request.")));
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		//Perform NDT if model is not still empty
		double score = 1;
		model_mutex.lock();
		double model_size = targetModel->width * targetModel->height;
		model_mutex.unlock();
		if (!model_size == 0) {
			score = testNDT(cloud,0.05);
		}
		else {
			std::cout << "Model is still empty, returning score of 1.\n";
		}
		//Send request back to Coordinator_ip/value with ndt fitness score
		std::stringstream ss2;
		ss2 << "http://" << coordinator_address << ":8080/" << utility::conversions::to_utf8string(value);
		const wchar_t* wc_coordinator_address = conversionsExtra::toWchar(ss2.str().c_str());
		http_client client(wc_coordinator_address);
		delete wc_coordinator_address;
		http_request msg;
		msg.set_method(methods::POST);
		msg.headers().add<double>(U("score"), score);
		// Send HTTP request
		try {
			client.request(msg).get();
			std::cout << "Sent request back to coordinator with NDT score.\n";
		}
		catch (const std::exception &e) {
			printf("Failed sending NDT score to coordinator Error exception:%s\n", e.what());
		}
	});
	ndt_listener->open().wait();
	std::cout << "***Listening for NDT requests from coordinator...\n";
}

template <class T>
void ClusterRESTHandler<T>::startListeningClouds() {
	std::stringstream ss;
	ss << "http://*:8080/" << cluster_id << "/cloud";
	const wchar_t* wc_server_address = conversionsExtra::toWchar(ss.str().c_str());
	clouds_listener = new http_listener(wc_server_address);
	delete wc_server_address;
	clouds_listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "***Recieved cloud POST.\n";
		//extract client ID from header
		int client_id;
		if (request.headers().has(U("ID"))) {
			string_t client_idstr = request.headers().find(U("ID"))->second;
			try{
				client_id = atoi(utility::conversions::to_utf8string(client_idstr).c_str());
			}
			catch(...) {
				std::cout << "ERROR: Couldn't parse client_id from header string.\n";
				request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't parse client_id from header string.")));
				return;
			}
			std::cout << "***Recieved client's ID: " << client_id << std::endl;
		}
		else {
			std::cout << "ERROR: Couldn't determine client ID.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine client ID.")));
			return;
		}
		//check if it's a last message
		if (request.headers().has(U("DONE"))) {
			map<int, ClusterClient<T>*>::iterator it = clients.find(client_id);
			if (it == clients.end()) {
				std::cout << "WARNING: Recieved last message from client_id " << client_id << " that doesn't exist.\n";
				request.reply(web::http::status_codes::NotFound, utility::conversions::to_string_t(std::string("Client ID from last message doesn't exist.")));
				return;
			}
			else {
				it->second->recievedAllFrames();
				//delete it->second;
				clients.erase(client_id);
				std::cout << "***Succesfully recieved all frames from client " << client_id << std::endl;
				request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("Succesfully recieved last message.")));
				return;
			}
		}
		//extract cloud from body
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "***Recieved client pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		//write stream to file cloud_[client_id]_[framenumber].pcd
		clients_mutex.lock();
		map<int, ClusterClient<T>*>::iterator it = clients.find(client_id);
		int frame_number;
		bool new_client;
		ClusterClient<T>* client;
		if (it == clients.end()) {
			//new client -> extract current ICP-model and create new ClusterClient
			std::cout << "***Client ID " << client_id << " is a new client.\n";
			client = new ClusterClient<T>(client_id,this);
			clients.insert(pair<int,ClusterClient<T>*>(client_id,client));
			clients_queue_mutex.lock();
			clients_queue.push(client);
			clients_queue_mutex.unlock();
			queue_condition.notify_one();
			//boost::this_thread::yield();
			frame_number = -1;
			new_client = true;
		}
		else {
			//existing client
			client = it->second;
			frame_number = client->get_frames_saved();
			new_client = false;
		}
		clients_mutex.unlock();
		std::ofstream output;
		std::stringstream ss;
		ss << "cloud_" << client_id << "_" << frame_number << ".pcd";
		std::string file_name = ss.str();
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			std::cout << "***Writing to file " << file_name << std::endl;
			output.write(reinterpret_cast<char *>(text), size);
		}
		std::cout << "***Wrote to file \n";
		output.close();
		//read file cloud_[client_id]_[framenumber].pcd to PointCloud
		pcl::PointCloud<T>::Ptr cloud (new pcl::PointCloud<T>());
		if (pcl::io::loadPCDFile<T>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
			return;
		}
		std::cout << "***Read PointCloud from " << file_name << std::endl;
		request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("Succesfully recieved frame.")));
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		if (!new_client) {
			client->addFrame(cloud);
			std::cout << "***Added frame to client's ICP-queue, client's queuesize: " << client->get_queue_size() << std::endl;
		}
		else {
			if (targetModel->size() == 0) {
				*targetModel = *cloud;
				client->setNDTTransformation(Eigen::Matrix4f::Identity());
			} 
			else {
			pcl::NormalDistributionsTransform<T,T>::Ptr ndt = registerModel(cloud,0.05);
			client->setNDTTransformation(ndt->getFinalTransformation());
			}
		}
	});
	clouds_listener->open().wait();
	std::cout << "***Listening for client frames...\n";
}

template <class T>
void ClusterRESTHandler<T>::startListeningViewRequests() {
	clouds_listener->support(web::http::methods::GET, [this](web::http::http_request request) {
		std::cout << "***Recieved view request.\n";
		std::string file_name("model_request.pcd");
		model_mutex.lock();
		save_to_file(targetModel,file_name.c_str());
		std::cout << "Model contains " << targetModel->height * targetModel->width << " data points.\n";
		model_mutex.unlock();
		std::streampos size;
		char * memblock;
		std::ifstream file (file_name, std::ios::in|std::ios::binary|std::ios::ate);
		if (file.is_open())
		{
			size = file.tellg();
			memblock = new char [size];
			file.seekg (0, std::ios::beg);
			file.read (memblock, size);
			file.close();
			std::cout << "Read file.\n";
		} 
		else std::cout << "ERROR: Unable to open file " << string(file_name);

		size_t bufferSize = size;
		// Put text to the buffer
		concurrency::streams::producer_consumer_buffer<char> buffer;
		buffer.putn((const char*)memblock, bufferSize).wait();
		buffer.close(std::ios_base::out);
		std::cout << "Put in buffer.\n";
		//Send response back to client with model
		http_response response;
		response.set_body(concurrency::streams::istream(buffer));
		response.set_status_code(web::http::status_codes::OK);
		response.headers().set_content_length(size);
		request.reply(response);
		//TEST
		/*
		PointCloudPtr test (new PointCloud);
		if (pcl::io::loadPCDFile<T>(file_name, *test) == -1) //* load the file
		{
			std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
			return;
		}
		visualizeCloud(test);
		*/
		//Delete file
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
	});
	clouds_listener->open().wait();
	std::cout << "***Listening for view requests...\n";
}

template <class T>
double ClusterRESTHandler<T>::testNDT(const PointCloudPtr inputCloud, float downSampleLeafSize) {
	std::cout << "Testing NDT.\n";
	PointCloudPtr filteredCloud (new PointCloud);
	if (downSampleLeafSize != 0) {
		filteredCloud = filterfunctions::downSample<T>(inputCloud,downSampleLeafSize);
	}
	else {
		filteredCloud = inputCloud;
	}
	pcl::NormalDistributionsTransform<T,T>::Ptr ndt (new pcl::NormalDistributionsTransform<T,T>);
  ndt->setTransformationEpsilon (0.01);
  ndt->setStepSize (0.1);
  ndt->setResolution (1.0);
  ndt->setMaximumIterations (30);
  ndt->setInputSource (filteredCloud);
	model_mutex.lock();
  ndt->setInputTarget (targetModel);
	PointCloudPtr outputCloud (new PointCloud);
	ndt->align(*outputCloud);
	model_mutex.unlock();
	PCL_INFO("***NDT*** Normal Distributions Transform completed. converged: (%d) iterations: (%d) score: (%f) probability: (%f)\n",ndt->hasConverged(), ndt->getFinalNumIteration(), ndt->getFitnessScore(), ndt->getTransformationProbability());
	return ndt->getFitnessScore();
}

template <class T>
typename pcl::NormalDistributionsTransform<T,T>::Ptr ClusterRESTHandler<T>::registerModel(const PointCloudPtr inputCloud, float downSampleLeafSize) {
	std::cout << "Registering model to targetModel with NDT.\n";
	PointCloudPtr filteredCloud (new PointCloud);
	if (downSampleLeafSize != 0) {
		filteredCloud = filterfunctions::downSample<T>(inputCloud,downSampleLeafSize);
	}
	else {
		filteredCloud = inputCloud;
	}
	pcl::NormalDistributionsTransform<T,T>::Ptr ndt (new pcl::NormalDistributionsTransform<T,T>);
  ndt->setTransformationEpsilon (0.01);
  ndt->setStepSize (0.1);
  ndt->setResolution (1.0);
  ndt->setMaximumIterations (30);
  ndt->setInputSource (filteredCloud);
	model_mutex.lock();
  ndt->setInputTarget (targetModel);
	PointCloudPtr outputCloud (new PointCloud);
	ndt->align(*outputCloud);
	*targetModel += *outputCloud;
	model_mutex.unlock();
	PCL_INFO("***NDT*** Normal Distributions Transform completed. converged: (%d) iterations: (%d) score: (%f) probability: (%f)\n",ndt->hasConverged(), ndt->getFinalNumIteration(), ndt->getFitnessScore(), ndt->getTransformationProbability());
	return ndt;
}

template <class T>
void ClusterRESTHandler<T>::addCloud(PointCloudPtr inputCloud) {
	//filterfunctions::downSample<T>(inputCloud, 0.015);
	model_mutex.lock();
	*targetModel += *inputCloud;
	targetModel = filterfunctions::downSample<T>(targetModel, 0.01);
	//std::cout << "ADDING CLOUD TO MODEL, MODEL SIZE: " << targetModel->height * targetModel->width << std::endl;
	model_mutex.unlock();
}

template <class T>
void ClusterRESTHandler<T>::save_to_file(PointCloudPtr cloud, const char* file_name) {
	pcl::io::savePCDFileBinary(file_name, *cloud);
}

template <class T>
void ClusterRESTHandler<T>::viewModel() {
	visualizeCloud(targetModel);
}

template <class T>
void ClusterRESTHandler<T>::visualizeCloud(PointCloudPtr cloud) {
	std::cout << "Visualizing cloud with " << cloud->width * cloud->height << " data points.\n";
	pcl::visualization::PCLVisualizer::Ptr visualizer (new pcl::visualization::PCLVisualizer("Model"));
	visualizer->addPointCloud(cloud);
	visualizer->addCoordinateSystem();
	visualizer->initCameraParameters();
	visualizer->spin();
}

//#endif