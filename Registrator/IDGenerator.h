#ifndef __IDGENERATOR_H_INCLUDED__
#define __IDGENERATOR_H_INCLUDED__

class IDGenerator {
private:
	static int client_id;
	static int cluster_id;
	static int listener_value;
public:
	static int generateClientID() {client_id++; return client_id;};
	static int generateClusterID() {cluster_id++; return cluster_id;};
	static int generateListenerValue() {listener_value++; return listener_value;};
};

#endif