#include "Graph.h"

void Graph::Graph(int capacity)
{
	m_Capacity = capacity;
	m_Countnumber = 0;
	m_nodeArray = new Node[m_Capacity];
	memset(m_AdjacentMatrix,0,m_Capacity*m_Capacity*sizeof(int));
}
void Graph::~Graph()
{
	delete []m_Nodelist;
	delete []m_AdjacentMatrix;
}
void Graph::AddNode(Node *pNode)
{
	if(pNode == NULL)
	{
		return false;
	}
	m_nodeArray[m_Countnumber].m_data = pNode->m_data;
	m_Countnumber ++;
	return true;
}
void Graph::ResetNode()
{
	for (int i = 0; i < m_Capacity; ++i)
	{
		m_Nodelist[i].m_Isvisited = false;
	}
}
bool Graph::SetValueToDirectedGraph(int row, int col, int value)
{
	if(row>=m_Capacity||row<0||col>=m_Capacity||col<0)
	{
		return false;
	}
	m_AdjacentMatrix[row*m_Capacity+col] = value;
	return true;
}
bool Graph::SetValueToUnDirectedGraph(int row, int col, int value)
{
	if(row>=m_Capacity||row<0||col>=m_Capacity||col<0)
	{
		return false;
	}
	m_AdjacentMatrix[row*m_Capacity+col] = value;
	return true;
}
bool Graph::getValueFromMtrix(int row, int col, int& val)
{	
	if(row>=m_Capacity||row<0||col>=m_Capacity||col<0)
	{
		return false;
	}
	val = m_AdjacentMatrix[row*m_Capacity+col];
	return true;
}
void Graph::printfMatrix()
{
	for(int row=0;row<m_Capacity;++row)
	{
		for(int col=0;col<m_Capacity;++col)
		{
			printf("%f  ",m_AdjacentMatrix[row*m_Capacity+col] );
		}
		printf("\n");
	}
}