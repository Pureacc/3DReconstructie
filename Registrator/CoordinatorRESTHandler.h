#ifndef __CLIENTRESTHANDLER_H_INCLUDED__
#define __CLIENTRESTHANDLER_H_INCLUDED__

#include "stdafx.h"
#include "ClusterTree.h"
#include "IDGenerator.h"
#include "ConversionsExtra.h"
#include "ClusterFunctions.h"
#include <map>
#include <queue>
using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace web::http::experimental::listener;

template <class T>
class CoordinatorRESTHandler {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<T> PointCloud;
private:
	//http_client* client;
	http_listener* client_listener;
	http_listener* split_listener;
	http_listener* client_update_listener;
	ClusterTree cluster_tree;
	std::map<int,std::string> cluster_addresses; //maps Cluster ID to IP address
	boost::mutex mutex_cluster_addresses;
	std::queue<std::string> available_ips;
	boost::mutex mutex_available_ips;

	void save_to_file(typename pcl::PointCloud<T>::Ptr cloud, const char* file_name);
	int tryClustersNDT(const char* file_name); //iterates all clusters and returns cluster_id of best NDT match, should return -1 if no match is found
	int checkBetterCluster(int current_cluster_id, const char* file_name); //returns different cluster id if client has moved to the area of this new cluster
	double tryClusterNDT(int cluster_id, const char* file_name); //returns fitnessScore 
	void startListeningCluster(int listener_value, double &score);
	std::string getAvailableIP();
public:
	void startListeningClients();
	void startListeningSplits();
	void startListeningClientUpdates(); //checks if a client is moving to an area of a different cluster, and returns the new address if so.
	void addAvailableIP(std::string ip);
};

template <class T>
void CoordinatorRESTHandler<T>::addAvailableIP(std::string ip) {
	mutex_available_ips.lock();
	available_ips.push(ip);
	mutex_available_ips.unlock();
}

template <class T>
std::string CoordinatorRESTHandler<T>::getAvailableIP() {
	mutex_available_ips.lock();
	std::string ip = available_ips.front();
	available_ips.pop();
	mutex_available_ips.unlock();
	return ip;
}

template <class T>
void CoordinatorRESTHandler<T>::startListeningCluster(int listener_value, double &score) {
	bool recieved_response = false;
	boost::mutex mutex;
	std::stringstream uriss;
	uriss << "http://*:8080/" << listener_value;
	http_listener cluster_listener(utility::conversions::to_string_t(uriss.str()));
	cluster_listener.support(web::http::methods::POST, [&recieved_response,&mutex,&score](web::http::http_request request) {
		std::cout << "Recieved cluster NDT response.\n";
		//extract character stream from body and client IP from header
		if (request.headers().has(U("score"))) {
			string_t score_t = request.headers().find(U("score"))->second;
			score = atof(utility::conversions::to_utf8string(score_t).c_str());
			std::cout << "Recieved cluster's NDT score: " << score << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine NDT score from cluster.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine score")));
			return;
		}
		//reply
		http_response response;
		response.set_status_code(web::http::status_codes::OK);
		response.set_body(utility::conversions::to_string_t(std::string("POST recieved")));
		request.reply(response);
		//
		mutex.lock();
		recieved_response = true;
		mutex.unlock();
		
	});
	cluster_listener.open().wait();
	std::cout << "Listening for cluster...\n";
	mutex.lock();
	while (!recieved_response) {
		mutex.unlock();
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		mutex.lock();
	}
	mutex.unlock();
	cluster_listener.close().wait();
}

template <class T>
double CoordinatorRESTHandler<T>::tryClusterNDT(int cluster_id, const char* file_name) {
	double score = 1;
	mutex_cluster_addresses.lock();
	std::string cluster_address = cluster_addresses.find(cluster_id)->second;
	mutex_cluster_addresses.unlock();
	std::cout << "Requesting NDT with cluster " << cluster_id << " on address " << cluster_address << std::endl;
	std::stringstream uriss;
	uriss << "http://" << cluster_address << ":8080/" << cluster_id << "/ndt";
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
		std::cout << "Read file.\n";
	} 
	else std::cout << "ERROR: Unable to open file " << file_name;
	size_t bufferSize = size;
	// Put text to the buffer
	concurrency::streams::producer_consumer_buffer<char> buffer;
	buffer.putn((const char*)memblock, bufferSize).wait();
	buffer.close(std::ios_base::out);
	std::cout << "Put in buffer.\n";
	// Create HTTP request
	int listener_value = IDGenerator::generateListenerValue();
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().set_content_length(size);
	msg.headers().add<int>(U("value"),listener_value);
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
		return -1;
	}
	std::cout << "Request sent.\n";
	bool recieved_response = false;
	boost::mutex mutex;
	startListeningCluster(listener_value, score); //returns when score has been recieved
	return score;
}

template <class T>
int CoordinatorRESTHandler<T>::checkBetterCluster(int current_cluster_id, const char* file_name) {
	int cluster_id;
	double best_score = 1;
	//get fitnessscore for current cluster
	double current_score = tryClusterNDT(current_cluster_id,file_name);
	mutex_cluster_addresses.lock();
	std::map<int,std::string>::iterator it = cluster_addresses.begin();
	while (it != cluster_addresses.end()) {
		int id = it->first;
		mutex_cluster_addresses.unlock();
		if (id != current_cluster_id) {
			double score = tryClusterNDT(id, file_name);
			std::cout << "Recieved score " << score << " from cluster " << id << std::endl;
			if (score < best_score) {
				best_score = score;
				cluster_id = id;
			}
		}
		mutex_cluster_addresses.lock();
		it++;
	}
	mutex_cluster_addresses.unlock();
	if (best_score < current_score) {
		return cluster_id;
	}
	else {
		return current_cluster_id;
	}
}

template <class T>
int CoordinatorRESTHandler<T>::tryClustersNDT(const char* file_name) {
	std::cout << "Trying to match client model with clusters\n";
	int cluster_id;
	double best_score = 1;
	mutex_cluster_addresses.lock();
	std::map<int,std::string>::iterator it = cluster_addresses.begin();
	while (it != cluster_addresses.end()) {
		int id = it->first;
		mutex_cluster_addresses.unlock();
		double score = tryClusterNDT(id, file_name);
		std::cout << "Recieved score " << score << " from cluster " << id << std::endl;
		if (score < best_score) {
			best_score = score;
			cluster_id = id;
		}
		mutex_cluster_addresses.lock();
		it++;
	}
	mutex_cluster_addresses.unlock();
	std::cout << "Finished matching client model with clusters\n";
	return cluster_id;
}

template <class T>
void CoordinatorRESTHandler<T>::startListeningClients() {
	client_listener = new http_listener(L"http://*:8080/register");
	client_listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "Recieved register POST.\n";
		//extract character stream from body and client IP from header
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "Recieved register pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		string_t client_ip_address;
		if (request.headers().has(U("IPAddress"))) {
			client_ip_address = request.headers().find(U("IPAddress"))->second;
			std::cout << "Recieved client's IP-address: " << utility::conversions::to_utf8string(client_ip_address) << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine client IP-address.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine IP address")));
			return;
		}
		//generate new client ID
		int client_id = IDGenerator::generateClientID();
		//write stream to file cloud_[client_id]
		std::ofstream output;
		std::stringstream ss;
		ss << "cloud_" << client_id << ".pcd";
		std::string file_name = ss.str();
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			std::cout << "Writing to file " << file_name << std::endl;
			output.write(reinterpret_cast<char *>(text), size);
		}
		std::cout << "Wrote to file \n";
		output.close();
		//read file cloud_[client_id] to PointCloud
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
			return;
		}
		std::cout << "Read PointCloud from " << file_name << std::endl;
		//add client ID to response and reply
		http_response response;
		response.set_status_code(web::http::status_codes::OK);
		response.set_body(utility::conversions::to_string_t(std::string("POST recieved")));
		request.reply(response);
		int cluster_id;
		std::string address;
		//if clusters is empty, create new cluster (send to cluster_address/create) with empty targetModel and return this address to client
		mutex_cluster_addresses.lock();
		bool isEmpty = cluster_addresses.empty();
		mutex_cluster_addresses.unlock();
		if (isEmpty) {
			std::cout << "No clusters yet available, creating new cluster.\n";
			address = getAvailableIP();
			addAvailableIP(address); //Put back in queue to create a round-robin effect
			cluster_id = IDGenerator::generateClusterID();
			mutex_cluster_addresses.lock();
			cluster_addresses.insert(std::pair<int,std::string>(cluster_id,address));
			mutex_cluster_addresses.unlock();
			//Send request to server to create new cluster
			std::stringstream ss_create;
			ss_create << "http://" << address << ":8080/create";
			const wchar_t* wc_server_address = conversionsExtra::toWchar(ss_create.str().c_str());
			http_client client(wc_server_address);
			delete wc_server_address;
			http_request msg;
			msg.set_method(methods::POST);
			msg.headers().add<int>(U("Cluster_ID"), cluster_id);
			try {
				client.request(msg).get();
				std::cout << "Sent request to create new cluster with ID " << cluster_id << ".\n";
			}
			catch (const std::exception &e) {
				printf("Failed sending request to create new cluster with ID %d Error exception:%s\n", cluster_id, e.what());
			}
			boost::this_thread::sleep_for(boost::chrono::seconds(1)); //Give some time for cluster to get ready
		}
		else {
			//try NDT for each cluster in the cluster_tree and determine best cluster
			cluster_id = tryClustersNDT(file_name.c_str());
			mutex_cluster_addresses.lock();
			address = cluster_addresses.find(cluster_id)->second;
			mutex_cluster_addresses.unlock();
		}
		//remove file after sent to clusters
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		//std::string cluster_address("http://192.168.1.2:8080/cloud/"); //TO DO THUIS
		//std::string cluster_address("http://10.10.131.41:8080/cloud/"); //TO DO IBCN
		//std::string cluster_address("http://192.168.0.170:8080/cloud/"); //TO DO KOT
		bool found_ndt;
		found_ndt = true;
		if (found_ndt) {
			//make request (POST) to client with URI of best cluster as body, client_id, Registered TRUE
			std::stringstream uriss;
			uriss << "http://" << utility::conversions::to_utf8string(client_ip_address) << ":8080/registerresponse";
			http_client client(utility::conversions::to_string_t(uriss.str()));
			http_request msg;
			std::stringstream clusteraddress;
			clusteraddress << "http://" << address << ":8080/" << cluster_id << "/cloud/";
			//msg.set_body(cluster_address);
			msg.set_body(clusteraddress.str());
			msg.set_method(methods::POST);
			msg.headers().add<int>(U("Client_id"), client_id);
			msg.headers().add(U("Registered"), U("TRUE"));
			// Send HTTP request
			try {
				client.request(msg).get();
				std::cout << "Sent request back to client with client_id and cluster IP.\n";
			}
			catch (const std::exception &e) {
				printf("Failed sending cluster URI to client %d Error exception:%s\n", client_id, e.what());
			}
		}
		else {
			//make request (POST) to client with Registered FALSE
			std::stringstream uriss;
			uriss << "http://" << utility::conversions::to_utf8string(client_ip_address) << ":8080/";
			http_client client(utility::conversions::to_string_t(uriss.str()));
			http_request msg;
			msg.set_method(methods::POST);
			msg.headers().add(U("Registered"), U("FALSE"));
			// Send HTTP request
			try {
				client.request(msg).get();
				std::cout << "Sent request back to client with cluster IP.\n";
			}
			catch (const std::exception &e) {
				printf("Failed sending cluster URI to client %d Error exception:%s\n", client_id, e.what());
			}
		}
	});
	client_listener->open().wait();
	std::cout << "Listening for clients...\n";
}

//Expects client frame/model and current cluster-address to compare to, and client IP-address + client_ID. Returns new cluster-address (in new http-request)
template <class T>
void CoordinatorRESTHandler<T>::startListeningClientUpdates() {
	client_update_listener = new http_listener(L"http://*:8080/clientupdate");
	client_update_listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "Recieved client update request.\n";
		//extract character stream from body and client IP from header
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "Recieved client update pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		string_t client_ip_address;
		if (request.headers().has(U("IPAddress"))) {
			client_ip_address = request.headers().find(U("IPAddress"))->second;
			std::cout << "Recieved client's IP-address: " << utility::conversions::to_utf8string(client_ip_address) << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine client IP-address.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine IP address")));
			return;
		}
		string_t clusterREST_t;
		if (request.headers().has(U("clusterREST"))) {
			clusterREST_t = request.headers().find(U("clusterREST"))->second;
			std::cout << "Recieved client's cluster REST address: " << utility::conversions::to_utf8string(clusterREST_t) << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine client's cluster REST address.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine cluster REST address")));
			return;
		}
		string_t client_id_t;
		if (request.headers().has(U("clientID"))) {
			client_id_t = request.headers().find(U("clientID"))->second;
			std::cout << "Recieved client ID: " << utility::conversions::to_utf8string(client_id_t) << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine client ID.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine client ID")));
			return;
		}
		//write stream to file updatecloud_[client_id].pcd
		std::string client_id = utility::conversions::to_utf8string(client_id_t);
		std::ofstream output;
		std::stringstream ss;
		ss << "updatecloud_" << client_id << ".pcd";
		std::string file_name = ss.str();
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			std::cout << "Writing to file " << file_name << std::endl;
			output.write(reinterpret_cast<char *>(text), size);
		}
		output.close();
		//read file cloud_[client_id] to PointCloud
		PointCloudPtr cloud (new PointCloud);
		if (pcl::io::loadPCDFile<T>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
			return;
		}
		std::cout << "Read PointCloud from " << file_name << std::endl;
		//reply
		http_response response;
		response.set_status_code(web::http::status_codes::OK);
		response.set_body(utility::conversions::to_string_t(std::string("POST recieved")));
		request.reply(response);
		//parse current cluster_id
		std::string clusterREST = utility::conversions::to_utf8string(clusterREST_t);
		int pos1 = clusterREST.find('/',10);
		int pos2 = clusterREST.find('/',pos1+1);
		std::string str = clusterREST.substr(pos1+1,pos2-pos1-1);
		int current_cluster_id;
		try {
			current_cluster_id = std::stoi(str);
		}
		catch (std::invalid_argument &e) {
			std::cout << "ERROR: Couldn't parse cluster ID from cluster REST address.\n";
			return;
		}
		//try NDT for each cluster in the cluster_tree except for the current one, and determine best cluster (can be the current one)
		int cluster_id;
		std::string address;
		cluster_id = checkBetterCluster(current_cluster_id, file_name.c_str());
		mutex_cluster_addresses.lock();
		address = cluster_addresses.find(cluster_id)->second;
		mutex_cluster_addresses.unlock();	
		//remove file after sent to clusters
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		//make request (POST) to client with URI of best cluster as body
		std::stringstream uriss;
		uriss << "http://" << utility::conversions::to_utf8string(client_ip_address) << ":8080/updateresponse";
		http_client client(utility::conversions::to_string_t(uriss.str()));
		http_request msg;
		std::stringstream clusteraddress;
		clusteraddress << "http://" << address << ":8080/" << cluster_id << "/cloud/";
		//msg.set_body(cluster_address);
		msg.set_body(clusteraddress.str());
		msg.set_method(methods::POST);
		// Send HTTP request
		try {
			client.request(msg).get();
			std::cout << "Sent request back to client with cluster REST address.\n";
		}
		catch (const std::exception &e) {
			printf("Failed sending cluster REST address to client %d Error exception:%s\n", client_id, e.what());
		}
	});
	client_update_listener->open().wait();
	std::cout << "Listening for client update requests...\n";
}

//expects model and cluster ID
template <class T>
void CoordinatorRESTHandler<T>::startListeningSplits() {
	split_listener = new http_listener(L"http://*:8080/split");
	split_listener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "Recieved split request.\n";
		//extract character stream from body and client IP from header
		auto bodyStream = request.body();
		concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
		long size = request.headers().content_length();
		std::cout << "Recieved split pointcloud with size " << size << std::endl;
		uint8_t* text = new uint8_t[size];
		size_t returned = buf.getn(text,size).get();
		string_t cluster_id;
		if (request.headers().has(U("ID"))) {
			cluster_id = request.headers().find(U("ID"))->second;
			std::cout << "Recieved cluster's ID: " << utility::conversions::to_utf8string(cluster_id) << std::endl;
		}
		else {
			std::cout << "ERROR: Coordinator couldn't determine cluster ID.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("Couldn't determine ID")));
			return;
		}
		int cluster_id_int = atoi(utility::conversions::to_utf8string(cluster_id).c_str());
		mutex_cluster_addresses.lock();
		std::map<int,std::string>::iterator it = cluster_addresses.find(cluster_id_int);
		if (it == cluster_addresses.end()) {
			mutex_cluster_addresses.unlock();
			std::cout << "ERROR: Recieved cluster ID " << utility::conversions::to_utf8string(cluster_id) << " doesn't exist.\n";
			request.reply(web::http::status_codes::PartialContent,utility::conversions::to_string_t(std::string("ID doesn't exist")));
			return;
		}
		mutex_cluster_addresses.unlock();
		//addAvailableIP(it->second);
		int second_id;
		//cluster_tree.split(cluster_id_int, first_id, second_id);
		std::string first_ip = it->second;
		std::string second_ip = getAvailableIP();
		addAvailableIP(second_ip);
		second_id = IDGenerator::generateClusterID();
		mutex_cluster_addresses.lock();
		cluster_addresses.insert(std::pair<int,std::string>(second_id, second_ip));
		mutex_cluster_addresses.unlock();
		//write stream to file model_[cluster_id].pcd
		std::ofstream output;
		std::stringstream ss;
		ss << "model_" << cluster_id_int << ".pcd";
		std::string file_name = ss.str();
		output.open(file_name, std::ios::out|std::ios::binary);
		if (output.is_open()) {
			output.write(reinterpret_cast<char *>(text), size);
		}
		output.close();
		//read file model_[cluster_id].pcd to PointCloud
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<T>(file_name, *cloud) == -1) //* load the file
		{
			std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
			request.reply(web::http::status_codes::InternalError,utility::conversions::to_string_t(std::string("Server couldn't read pointcloud file")));
			return;
		}
		std::cout << "Read PointCloud from " << file_name << std::endl;
		if(remove(file_name.c_str()) != 0)
			std::cout << "WARNING: failed deleting file " << file_name << std::endl;
		//reply
		http_response response;
		response.set_status_code(web::http::status_codes::OK);
		response.set_body(utility::conversions::to_string_t(std::string("Split recieved")));
		request.reply(response);
		//Perform split
		PointCloudPtr firstCloud (new PointCloud);
		PointCloudPtr secondCloud (new PointCloud);
		clusterfunctions::kMeansSplit<T>(cloud,firstCloud,secondCloud);
		save_to_file(firstCloud,"cluster_cloud_1.pcd");
		save_to_file(secondCloud,"cluster_cloud_2.pcd");
		//Send messages to the 2 ip-addresses with their split model
		//Message to original server address:8080/cluster_id/split
		std::stringstream ss_address1;
		ss_address1 << "http://" << first_ip << ":8080/" << cluster_id_int << "/split";
		const wchar_t* wc_server_address1 = conversionsExtra::toWchar(ss_address1.str().c_str());
		http_client client1(wc_server_address1);
		delete wc_server_address1;
		std::stringstream ss1;
		ss1 << "model" << cluster_id_int << "_first.pcd";
		file_name = ss1.str();
		save_to_file(firstCloud, file_name.c_str());
		std::cout << "Posting " << file_name << " to " << utility::conversions::to_utf8string(client1.base_uri().to_string()) << std::endl;
		std::streampos size1;
		char * memblock;
		std::ifstream file (file_name, std::ios::in|std::ios::binary|std::ios::ate);
		if (file.is_open())
		{
			size1 = file.tellg();
			memblock = new char [size1];
			file.seekg (0, std::ios::beg);
			file.read (memblock, size1);
			file.close();
		} 
		else std::cout << "ERROR: Unable to open file " << std::string(file_name);
		size_t bufferSize = size1;
		// Put text to the buffer
		concurrency::streams::producer_consumer_buffer<char> buffer;
		buffer.putn((const char*)memblock, bufferSize).wait();
		buffer.close(std::ios_base::out);
		// Create HTTP request
		http_request msg;
		msg.set_body(concurrency::streams::istream(buffer));
		msg.set_method(methods::POST);
		msg.headers().set_content_length(size1);
		int statusCode;
		// Send HTTP request
		try {
			http_response response = client1.request(msg).get();
			statusCode = response.status_code();
			delete memblock;
		}
		catch (const std::exception &e) {
			printf("Error exception:%s\n", e.what());
			delete memblock;
			return;
		}
		std::cout << "Request with 1st model sent.\n";

		//Message to new server address:8080/create
		std::stringstream ss_address2;
		ss_address2 << "http://" << second_ip << ":8080/create";
		const wchar_t* wc_server_address2 = conversionsExtra::toWchar(ss_address2.str().c_str());
		http_client client2(wc_server_address2);
		delete wc_server_address2;
		std::stringstream ss2;
		ss2 << "model" << cluster_id_int << "_second.pcd";
		file_name = ss2.str();  
		save_to_file(secondCloud, file_name.c_str());
		std::cout << "Posting " << file_name << " to " << utility::conversions::to_utf8string(client2.base_uri().to_string()) << std::endl;
		std::streampos size2;
		char * memblock2;
		std::ifstream file2 (file_name, std::ios::in|std::ios::binary|std::ios::ate);
		if (file2.is_open())
		{
			size2 = file2.tellg();
			memblock2 = new char [size2];
			file2.seekg (0, std::ios::beg);
			file2.read (memblock2, size2);
			file2.close();
		} 
		else std::cout << "ERROR: Unable to open file " << std::string(file_name);
		size_t bufferSize2 = size2;
		// Put text to the buffer
		concurrency::streams::producer_consumer_buffer<char> buffer2;
		buffer2.putn((const char*)memblock2, bufferSize2).wait();
		buffer2.close(std::ios_base::out);
		// Create HTTP request
		http_request msg2;
		msg2.set_body(concurrency::streams::istream(buffer2));
		msg2.set_method(methods::POST);
		msg2.headers().set_content_length(size2);
		msg2.headers().add<int>(U("Cluster_ID"), second_id);
		int statusCode2;
		// Send HTTP request
		try {
			http_response response = client2.request(msg2).get();
			statusCode2 = response.status_code();
			delete memblock2;
		}
		catch (const std::exception &e) {
			printf("Error exception:%s\n", e.what());
			delete memblock2;
			return;
		}
		std::cout << "Request with 2nd model sent.\n";
	});
	split_listener->open().wait();
	std::cout << "Listening for cluster splits...\n";
}

template <class T>
void CoordinatorRESTHandler<T>::save_to_file(PointCloudPtr cloud, const char* file_name) {
	pcl::io::savePCDFileBinary(file_name, *cloud);
}

#endif