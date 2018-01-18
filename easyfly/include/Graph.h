#ifndef GRAPH_H
#define GRAPH_H

#include "node.h"
class Graph
{
public:
	Graph(int Capacity);
	~Graph();
private:
	int m_Capacity;  //capacity of the graph
	int m_Countnumber;  //# of nodes that have already been put in the graph
	Node *m_nodeArray;  //List of nodes
	int *m_AdjacentMatrix; 

private:
	bool AddNode(Node *pNode);
	void ResetNode();
	bool SetValueToDirectedGraph(int col, int row, int value);
	bool SetValueToUnDirectedGraph(int col, int row, int value);
	bool getValueFromMtrix(int col, int row, int& val);
	void printfMatrix();
};
#endif