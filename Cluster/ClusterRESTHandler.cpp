#include "stdafx.h"
#include "ClusterRESTHandler.h"
#include "ClusterClient.h"

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
			std::cout << "WAITING FOR COND\n";
			queue_condition.wait(lock);
			std::cout << "COMPLETED WAITING FOR COND\n";
			lock.unlock();
		}
		ClusterClient<T>* client = getNextClient();
		//start client's ICP+NDT
		std::cout << "STARTING CLIENT REGISTER.\n";
		client->startRegistering();
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
		//Perform NDT
		double score = testNDT(cloud,0.05);
		//Send request back to Coordinator_ip/value with ndt fitness score
		std::stringstream ss;
		ss << "http://" << coordinator_address << ":8080/" << utility::conversions::to_utf8string(value);
		const wchar_t* wc_coordinator_address = conversionsExtra::toWchar(ss.str().c_str());
		http_client client(wc_server_address);
		delete wc_server_address;
		http_request msg;
		msg.set_method(methods::POST);
		msg.headers().add<double>(U("score"), score);
		// Send HTTP request
		try {
			client.request(msg).get();
			std::cout << "Sent request back to coordinator with NDT score.\n";
		}
		catch (const std::exception &e) {
			printf("Failed sending NDT score to coordinator %d Error exception:%s\n", client_id, e.what());
		}
	});
	clouds_listener->open().wait();
	std::cout << "***Listening for client frames...\n";
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
			std::cout << "NOTIFIED CONDITION \n";
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
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) //* load the file
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
			//client->addFrame(cloud);
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
void ClusterRESTHandler<T>::addCloud(const PointCloudPtr inputCloud) {
	model_mutex.lock();
	*targetModel += *inputCloud;
	model_mutex.unlock();
}