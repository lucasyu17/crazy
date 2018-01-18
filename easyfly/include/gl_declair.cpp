#include "gl_declair.h"
/*************YE Xin gl*/
extern void give_index(int index);
extern std::vector<float> x_init_pos;
extern std::vector<float> y_init_pos;
/*************YE Xin gl*/
// Enumeration
enum EnumDisplayMode { HIDDENLINE, FLATSHADED, SMOOTHSHADED, COLORSMOOTHSHADED, DELETESELECTEDVERTEX, WIREFRAME };

enum Mode
{
	Viewing,
	Selection
};
Mode currentMode = Viewing;


// variables
int displayMode = FLATSHADED;	// current display mode
int mainMenu, displayMenu;		// glut menu handlers
int winWidth, winHeight;		// window width and height
double winAspect;				// winWidth / winHeight;
int lastX, lastY;				// last mouse motion position
int currSelectedVertex = -1;         // current selected vertex
bool leftDown, leftUp, rightUp, rightDown, middleDown, middleUp, shiftDown;		// mouse down and shift down flags
double sphi = 90.0, stheta = 45.0, sdepth = 10;	// for simple trackball
double xpan = 0.0, ypan = 0.0;				// for simple trackball
double zNear = 1.0, zFar = 100.0;
double g_fov = 45.0;
//Vector3d g_center;
double g_sdepth;
//Mesh mesh;	// our mesh



void displayFunc()  
{  
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);// 在RGB模式下，使用glClearColor清空之后画布的颜色  
    glClear(GL_COLOR_BUFFER_BIT);// 清空画布  
    glColor3f(0.0f, 0.0f, 1.0f);
    glRectf(-0.8f, -0.8f, 0.8f, 0.8f);
    

 //   glLineStipple(2, 0x5555);
//glEnable(GL_LINE_STIPPLE);
    glColor3f(1.0,1.0,1.0);
	glBegin(GL_LINES);
	glVertex2f(0,0);
	glVertex2f(0,0.2);
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(0,0);
	glVertex2f(0.2,0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);
    glBegin(GL_TRIANGLES);                            // 绘制三角形
	glVertex2f( -0.02f, 0.2f);                    // 上顶点
	glVertex2f(0.02f,0.2f);                    // 左下
	glVertex2f( 0.0f,0.28f);                    // 右下
    glEnd(); 
    glBegin(GL_TRIANGLES);                            // 绘制三角形
	glVertex2f( 0.2f,-0.02f);                    // 上顶点
	glVertex2f(0.2f,0.02f);                    // 左下
	glVertex2f( 0.28f,0.0f);                    // 右下
    glEnd(); 
    

	char text[] = {'x','y'};

	glRasterPos2f(0.25,-0.1);

	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,text[0]);
	glRasterPos2f(-0.1,0.25);
    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,text[1]);

    glColor3f(1.0f, 0.0f, 0.0f);// 设置画笔颜色 
    glPointSize(5.0f);// 设置点的大小 
    glBegin(GL_POINTS);
    printf("\n");
    for(int i=0;i<x_init_pos.size();i++){
    	glVertex2f(x_init_pos[i], y_init_pos[i]);  
    	printf("x%d: %f\n", i, x_init_pos[i]);
    	printf("y%d: %f\n", i, y_init_pos[i]);
    	fflush(stdout);
    }
 //   glVertex2f(-0.5f, -0.5f);  
    glEnd();
    // do somethins  
    glFlush();// 清空缓冲区，立即执行绘制命令  
} 


void DrawSelectedVertices(int i)
{
//	DrawFlatShaded();
//	DisplayFunc();
	if (currSelectedVertex != -1)
	{
	//	VertexList vList = mesh.Vertices();
/*************YE Xin gl*/
		glPointSize(10.0f);
		glColor3f(1.0, 0.1, 0.1);
		glBegin(GL_POINTS);
		glVertex2f(x_init_pos[i], y_init_pos[i]);
//		glVertex3dv(vList[currSelectedVertex]->Position().ToArray());
		glEnd();
/*************YE Xin gl*/
	}
	else
	{
		//	cout << "You haven't selected any vertex" << endl;
	}
	glDisable(GL_LIGHTING);
//	cout << "the selected vertex is drawn " << endl;
}


// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {

	
	lastX = x;
	lastY = y;
	float x_trans=(x-200)/200.0f;
	float y_trans=(-y+200)/200.0f;
	leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
	leftUp = (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP);
	rightDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
	rightUp = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_UP);
	middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
	middleUp = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_UP);
//	shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);

	if (state == GLUT_UP)
	{
		if (middleUp)//middle button
		{
			if (currSelectedVertex != -1)
			{
//				mesh.Vertices()[currSelectedVertex]->SetFlag(0);
//				currSelectedVertex = -1;
			}
		}
		else{
			printf("x:%f\n", x_trans);
			printf("y:%f\n", y_trans);
			fflush(stdout);
/*************YE Xin gl*/
		//	SelectVertexByPoint();
		//	DrawSelectedVertices();
			float nearest_dist=-1.0f;
			int nearest_index=0;
			for(int i=0;i<x_init_pos.size();i++){
				float sq_dist=(x_trans-x_init_pos[i]*1000)*(x_trans-x_init_pos[i]*1000)+(y_trans-y_init_pos[i]*1000)*(y_trans-y_init_pos[i]*1000);
				if(sq_dist<nearest_dist||nearest_dist<0){
					nearest_dist=sq_dist;
					nearest_index=i;
				}
			}
			give_index(nearest_index);
    	
/*************YE Xin gl*/
		}

		lastX = lastY = 0;
	}

//	index_sequence[i]
}