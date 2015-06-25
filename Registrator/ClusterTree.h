#ifndef __CLUSTERTREE_H_INCLUDED__
#define __CLUSTERTREE_H_INCLUDED__
#include <iostream>
#include "IDGenerator.h"
#include <set>

struct Node;

class ClusterTree {

public:
	ClusterTree() {k=0;};
	~ClusterTree();
	void split(int Id, int &firstId, int &secondId);
	Node* search(int Id);
	int initialize();
	int size();
	void setNode(Node* node_);
	std::set<int> getClusterIds(); //returns the ids in all leaves

private:
	Node* k;

	void search(int Id, Node* &node);
	void recursive_addLeaves(std::set<int> &ids);
};

struct Node {
	int Id;
	bool isLeaf;
	ClusterTree left, right;
	Node* parent;
};

std::set<int> ClusterTree::getClusterIds() {
	std::set<int> ids;
	recursive_addLeaves(ids);
	return ids;
}

void ClusterTree::recursive_addLeaves(std::set<int> &ids) {
	if (k->isLeaf) {
		ids.insert(k->Id);
	}
	else {
		k->left.recursive_addLeaves(ids);
		k->right.recursive_addLeaves(ids);
	}
}

void ClusterTree::setNode(Node* node_) {
	if (k)
		delete k;
	k = node_;
}

ClusterTree::~ClusterTree() {
	if (k)
		delete k;
}

//Only use when creating new tree in main program
int ClusterTree::initialize() {
	k = new Node();
	k->Id = IDGenerator::generateClusterID();
	k->isLeaf = true;
	k->parent = 0;
	return k->Id;
}

int ClusterTree::size() {
	if (!k) {
		std::cout<<"IF"<<std::endl;
		return 0;
	}
	else {
		std::cout<<"ELSE "<<k->Id<<std::endl;
		return 1 + k->left.size() + k->right.size();
	}
}

void ClusterTree::split(int Id, int &firstId, int &secondId) {
	
	Node* node = search(Id);
	node->isLeaf = false;
	firstId = IDGenerator::generateClusterID();
	Node* first = new Node();
	first->Id = firstId;
	first->isLeaf = true;
	first->parent = node;
	node->left.setNode(first);
	
	secondId = IDGenerator::generateClusterID();
	Node* second = new Node();
	second->Id = secondId;
	second->isLeaf = true;
	second->parent = node;
	node->right.setNode(second);
	if (node->right.k) {
		std::cout<<"TRUE"<<std::endl;
	}
	else {
		std::cout<<"FALSE"<<std::endl;
	}
	if (node->left.k) {
		std::cout<<"TRUE"<<std::endl;
	}
	else {
		std::cout<<"FALSE"<<std::endl;
	}
}


Node* ClusterTree::search(int Id) {
	
	Node* node = new Node();
	if (k) {
		std::cout<<"SEARCHING"<<std::endl;
		search(Id,node);
	}
	return node;
	
}


void ClusterTree::search(int Id, Node* &node) {
	
	if (k) { //might be unnecessary
		if (k->isLeaf) {
			if (k->Id == Id) {
				std::cout<<"FOUND"<<std::endl;
				node = k;
			}
		}
		else {
			k->left.search(Id, node);
			k->right.search(Id, node);
		}
	}
	
}

#endif