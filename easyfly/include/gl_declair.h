#ifndef GL_H
#define GL_H

#include "glut.h"
#include <fstream>//20110819
#include <vector>
using namespace std;


/*void InitMenu();

void MenuCallback();*/

void displayFunc();

void DrawSelectedVertices();
void displayCrazyflie();

//void Partition();

void MouseFunc(int button, int state, int x, int y);
void SelectVertexByPoint();


#endif


/*******/
// select a mesh point

