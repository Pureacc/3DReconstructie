#ifndef __CLIENTRESTHANDLER_H_INCLUDED__
#define __CLIENTRESTHANDLER_H_INCLUDED__

#include "stdafx.h"

#include <string>
#include "ICPModelConstructor.h"
#include "ConversionsExtra.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
//#include "ICPModelConstructor.h"


using namespace std;
using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace web::http::experimental::listener;

template <class T>
class ClientRESTHandler {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<T> PointCloud;
private:
	http_client* client;
	http_listener* listener;
	http_listener* updateListener;
	ICPModelConstructor<T> model_constructor;
	string_t client_ID;
	const char* ip_address;
	const char* registrator_ip_address;
	bool sleep, found_ndt, updating;
	boost::thread* ndt_thread;

	void setBaseUri(const wchar_t* server_address); //Used to change server after registerToCoordinator returns machine to send frames to
	void startListening();
	void stopListening();
	void startListeningUpdate();
	void startSleep(); //Wait for response with cluster URI
	void stopSleep();
	void ndtLoop();
	void save_to_file(typename pcl::PointCloud<T>::Ptr cloud, const char* file_name);
	void visualize_cloud(typename pcl::PointCloud<T>::Ptr cloud);
	void postUpdateRequest(const char* file_name);
public:
	~ClientRESTHandler() {delete client;};
	//ClientRESTHandler();
	ClientRESTHandler(char* fileName, char* client_ip_address, char* server_IP);
	void startICPAndRegister();
	int registerToCoordinator(char* file_name); //Sends clientIP, Returns statuscode response, initializes clientID member, starts listener to recieve machine info
	int postFrame(const char* file_name); //Returns statuscode response
	int postModelRegistrator(const char* file_name);
	int postModelCluster(const char* file_name);
	int postLastMessage(); //Sends post to let cluster know there are no more frames
	void viewGlobalModel();
};

template <class T>
ClientRESTHandler<T>::ClientRESTHandler(char* file_name, char* client_ip_address, char* server_IP) : ip_address(client_ip_address), model_constructor(file_name) {
	updating = false;
	registrator_ip_address = server_IP;
	std::stringstream ss;
	ss << "http://" << server_IP << ":8080/register";
	const wchar_t* wc_server_address = conversionsExtra::toWchar(ss.str().c_str());
	setBaseUri(wc_server_address);
	delete wc_server_address;
}

template <class T>
void ClientRESTHandler<T>::viewGlobalModel() {
	std::cout << "Requesting to view model of " << utility::conversions::to_utf8string(client->base_uri().to_string()) << std::endl;
	// Create HTTP request
	http_request msg;
	msg.set_method(methods::GET);
	int statusCode;
	PointCloudPtr model (new PointCloud);
	// Send HTTP request
	http_response response;
	try {
		response = client->request(msg).get();
		//boost::this_thread::sleep(boost::posix_time::seconds(5));
		//visualize_cloud(model);
	}
	catch (const std::exception &e) {
		printf("Error exception:%s\n", e.what());
		return;
	}
	statusCode = response.status_code();
	//Extract body
	auto bodyStream = response.body();
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	concurrency::streams::streambuf<uint8_t> buf = bodyStream.streambuf();
	long size = response.headers().content_length();
	std::cout << "***Recieved cluster model with size " << size << std::endl;
	uint8_t* text = new uint8_t[size];
	size_t returned = buf.getn(text,size).wait();
	//Write to file
	std::ofstream output;
	std::stringstream ss;
	ss << "model_request.pcd";
	std::string file_name = ss.str();
	output.open(file_name, std::ios::out|std::ios::binary);
	if (output.is_open()) {
		std::cout << "***Writing to file " << file_name << std::endl;
		output.write(reinterpret_cast<char *>(text), size);
	}
	std::cout << "***Wrote to file \n";
	output.close();
	//read file
	if (pcl::io::loadPCDFile<T>(file_name, *model) == -1) //* load the file
	{
		std::cout << "ERROR: Couldn't read file " << file_name << std::endl;
		return;
	}
	std::cout << "***Read PointCloud from " << file_name << std::endl;
	visualize_cloud(model);
}

template <class T>
void ClientRESTHandler<T>::startICPAndRegister() {
	//startListeningUpdate();
	//startListeningUpdate();
	//start thread
	ndt_thread = new boost::thread(&ClientRESTHandler<T>::ndtLoop, this);
	//start building ICP-model, ndt thread sets flag if ndt found cluster
	model_constructor.buildModel(0.015);
	//Registrator has now found a cluster and base uri is set OR building has done all frames
	
	while (!found_ndt) {
		std::cout << "Waiting to recieve cluster address from registrator...\n";
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	PointCloudPtr model = model_constructor.getModel();
	save_to_file(model,"model.pcd");
	//visualize_cloud(model);
	postModelCluster("model.pcd");
	FrameReader<T> reader = model_constructor.getFrameReader();
	reader.decrementIndex();
	int count = 0;
	std::cout << "Sending remaining frames to cluster.\n";
	Eigen::Matrix4f global_transform = model_constructor.getGlobalTransform();
	boost::posix_time::ptime time_start(boost::posix_time::microsec_clock::local_time());
	listener->close().wait();
	while (reader.hasNextFrame()) { //send remaining frames to cluster after transformation
		PointCloudPtr cloud = reader.getNextFrame();
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
		cloud = filterfunctions::passThrough<PointT>(cloud,"z",1.0,3.0);
		PointCloudPtr transformed(new PointCloud);
		pcl::transformPointCloud(*cloud,*transformed,global_transform);
		std::stringstream ss;
		ss << "frame" << count << ".pcd";
		save_to_file(cloud, ss.str().c_str());
		postFrame(ss.str().c_str());
		count++;
		if (count%50 == 0 && !updating) {
			//postUpdateRequest(ss.str().c_str());
		}
	}
	boost::posix_time::ptime time_end(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration duration(time_end - time_start);
	std::cout << "Finished sending " << count << " remaining frames to cluster in " << duration << " seconds.\n";
	postLastMessage();
}

template <class T>
void ClientRESTHandler<T>::ndtLoop() {
	std::cout << "Starting NDT Loop.\n";
	found_ndt = false;
	PointCloudPtr model = model_constructor.getModel();
	while (model->width * model->height == 0) {
		boost::this_thread::sleep( boost::posix_time::seconds(1) );
		model = model_constructor.getModel();
	}
	while (!found_ndt) {
		save_to_file(model,"model.pcd");
		postModelRegistrator("model.pcd"); //sets client URI to cluster address when ndt is found
		//startListening();
		model = model_constructor.getModel();
	}
	//save_to_file(model,"model.pcd");
	//postModelCluster("model.pcd"); 
	model_constructor.stop();
	std::cout << "Stopped NDT Loop.\n";
}

template <class T>
void ClientRESTHandler<T>::save_to_file(typename pcl::PointCloud<T>::Ptr cloud, const char* file_name) {
	pcl::io::savePCDFileBinary(file_name, *cloud);
}

template <class T>
void ClientRESTHandler<T>::startSleep() {
	sleep = true;
	while (sleep) {
		boost::this_thread::sleep( boost::posix_time::seconds(1) );
	}
}

template <class T>
void ClientRESTHandler<T>::stopSleep() {
	sleep = false;
}

template <class T>
void ClientRESTHandler<T>::stopListening() {
	std::cout << "Stopping listener.\n";
	//listener->close();
	//std::cout << "Stopped listener.\n";
	stopSleep();
	//listener->close();
}

template <class T>
void ClientRESTHandler<T>::startListening() {
	std::cout << "Starting listener.\n";
	listener = new http_listener(L"http://*:8080/registerresponse");
	listener->support(web::http::methods::POST, [this](web::http::http_request request)
	{
		std::cout << "Recieved POST.\n";
		string_t registered = request.headers().find(U("Registered"))->second;
		if (utility::conversions::to_utf8string(registered).compare("TRUE") == 0) {
			std::cout << "Registrator found NDT match.\n";
			client_ID = request.headers().find(U("Client_id"))->second;
			wstring cluster_address;
			try {
				cluster_address = request.extract_string().get();
			}
			catch (...) {
				std::cout << "ERROR: Failed extracting cluster URI from request.\n";
			}

			setBaseUri(cluster_address.c_str());
			request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("POST Recieved")));
			std::cout << "Extracted cluster URI from request.\n";
			found_ndt = true;
		}
		else {
			std::cout << "Registrator has not found NDT match yet.\n";
			request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("POST Recieved")));
		}
		stopListening();
	});
	listener->open().wait();
	startSleep();
}

template <class T>
void ClientRESTHandler<T>::startListeningUpdate() {
	std::cout << "Starting listener for update.\n";
	//updateListener->close().wait();
	updateListener = new http_listener(L"http://*:8080/updateresponse");
	updateListener->support(web::http::methods::POST, [this](web::http::http_request request) {
		std::cout << "Recieved cluster update.\n";
		wstring cluster_address;
		try {
			cluster_address = request.extract_string().get();
		}
		catch (...) {
			std::cout << "ERROR: Failed extracting cluster URI from update response.\n";
		}
		setBaseUri(cluster_address.c_str());
		request.reply(web::http::status_codes::OK,utility::conversions::to_string_t(std::string("POST Recieved")));
		std::cout << "Updated cluster URI.\n";
		updating = false;
		updateListener->close().wait();
	});
	//std::cout << "Opening listener.\n";
	//updateListener->close().wait();
	//listener->close().wait();
	//std::cout << "Closed old listener.\n";
	updateListener->open().wait();
	boost::this_thread::sleep( boost::posix_time::milliseconds(200) );
	//boost::this_thread::sleep( boost::posix_time::seconds(1) );
	//std::cout << "Listener opened.\n";
}

template <class T>
void ClientRESTHandler<T>::setBaseUri(const wchar_t* server_address) {
	client = new http_client(server_address);
}

template <class T>
int ClientRESTHandler<T>::postFrame(const char* file_name) {
	std::cout << "Posting " << file_name << " to " << utility::conversions::to_utf8string(client->base_uri().to_string()) << std::endl;
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
	// Create HTTP request
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().add(U("ID"), client_ID);
	msg.headers().set_content_length(size);
	std::cout << "Built request.\n";
	int statusCode;
	// Send HTTP request
	auto requestTask = client->request(msg).then([&statusCode, &memblock](http_response response) {
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
	return statusCode;
}

template <class T>
int ClientRESTHandler<T>::postModelRegistrator(const char* file_name) {
	std::cout << "Posting " << file_name << " to registrator " << utility::conversions::to_utf8string(client->base_uri().to_string()) << std::endl;
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
	// Create HTTP request
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().set_content_length(size);
	msg.headers().add(U("IPAddress"), ip_address);
	std::cout << "Built request.\n";
	int statusCode;
	// Send HTTP request
	auto requestTask = client->request(msg).then([&statusCode, &memblock](http_response response) {
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
	startListening();
	return statusCode;
}

template <class T>
int ClientRESTHandler<T>::postModelCluster(const char* file_name) {
	std::cout << "Posting " << file_name << " to cluster " << utility::conversions::to_utf8string(client->base_uri().to_string()) << std::endl;
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
	// Create HTTP request
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().set_content_length(size);
	msg.headers().add(U("IPAddress"), ip_address);
	msg.headers().add(U("ID"), client_ID);
	std::cout << "Built request.\n";
	int statusCode;
	// Send HTTP request
	auto requestTask = client->request(msg).then([&statusCode, &memblock](http_response response) {
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
	return statusCode;
}

template <class T>
int ClientRESTHandler<T>::postLastMessage() {
	std::cout << "Posting last message to " << utility::conversions::to_utf8string(client->base_uri().to_string()) << std::endl;
	// Create HTTP request
	http_request msg;
	msg.set_method(methods::POST);
	msg.headers().add(U("ID"), client_ID);
	msg.headers().add(U("DONE"), utility::conversions::to_string_t(std::string("DONE")));
	int statusCode;
	// Send HTTP request
	auto requestTask = client->request(msg).then([&statusCode](http_response response) {
		statusCode = response.status_code();
	});
	try {
		requestTask.wait();
	}
	catch (const std::exception &e) {
		printf("WARNING: Failed posting last message, %s\n", e.what());
		return -1;
	}
	std::cout << "Request sent.\n";
	return statusCode;
}

template <class T>
int ClientRESTHandler<T>::registerToCoordinator(char* file_name) {
	std::cout << "Registering to " << utility::conversions::to_utf8string(client->base_uri().to_string()) << " with " << file_name << std::endl;
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
	}
	else std::cout << "ERROR: Unable to open file " << string(file_name);
	size_t bufferSize = size;
	// Put text to the buffer
	concurrency::streams::producer_consumer_buffer<char> buffer;
	buffer.putn((const char*)memblock, bufferSize).wait();
	buffer.close(std::ios_base::out);

	// Create HTTP request
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().set_content_length(size);
	msg.headers().add(U("IPAddress"), ip_address);

	int statusCode;
	wstring responseMessage;
	// Send HTTP request
	auto requestTask = client->request(msg).then([this, &statusCode, &responseMessage, &memblock](http_response response) {
		statusCode = response.status_code();
		responseMessage = response.extract_string().get();
		if (response.headers().has(U("ID"))) {
			web::http::http_headers::iterator it = response.headers().find(U("ID"));
			client_ID = it->second;
			std::cout << "Recieved client ID " << utility::conversions::to_utf8string(client_ID) << std::endl;
			delete memblock;
		}
		else {
			std::cout << "ERROR: Client didn't recieve ID value from " << utility::conversions::to_utf8string(this->client->base_uri().to_string()) << std::endl;
			delete memblock;
			statusCode = -1;
		}
	});
	try {
		requestTask.wait();
	}
	catch (const std::exception &e) {
		printf("Error exception:%s\n", e.what());
		delete memblock;
	}
	if (statusCode != web::http::status_codes::OK) {
		std::cout << "ERROR: recieved status code from coordinator: " << statusCode << " - " << std::string(responseMessage.begin(),responseMessage.end()) << std::endl;
		return statusCode;
	}
	std::cout << "Registered.\n";
	startListening();
	std::cout << "Ready to send frames.\n";
	return statusCode;
}

template <class T>
void ClientRESTHandler<T>::visualize_cloud(typename pcl::PointCloud<T>::Ptr cloud) {
	std::cout << "Recieved cluster model with " << cloud->width * cloud->height << " data points.\n";
	pcl::visualization::PCLVisualizer::Ptr visualizer (new pcl::visualization::PCLVisualizer("Model"));
	visualizer->addPointCloud(cloud);
	visualizer->addCoordinateSystem();
	visualizer->initCameraParameters();
	//visualizer->resetCameraViewpoint();
	visualizer->spin();
}

template <class T>
void ClientRESTHandler<T>::postUpdateRequest(const char* file_name) {
	std::cout << "Posting update request.\n";
	updating = true;
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
	// Create HTTP request
	http_request msg;
	msg.set_body(concurrency::streams::istream(buffer));
	msg.set_method(methods::POST);
	msg.headers().set_content_length(size);
	msg.headers().add(U("IPAddress"), ip_address);
	msg.headers().add(U("clientID"), client_ID);
	string_t uri = client->base_uri().to_string();
	msg.headers().add(U("ClusterREST"), uri);
	std::cout << "Built request.\n";
	int statusCode;
	// Send HTTP request
	std::stringstream ss;
	ss << "http://" << registrator_ip_address << ":8080/clientupdate";
	const wchar_t* wc_server_address = conversionsExtra::toWchar(ss.str().c_str());
	http_client client(wc_server_address);
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
	startListeningUpdate();
}

#endif