#ifndef NODE_H
#define NODE_H
class Node
{
public:
	Node(char* Data);
	~Node();
public:
	int m_index;
	char* m_data;
	bool m_Isvisited;
};
#endif