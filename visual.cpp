#include <GL/glut.h>

#include "struct.h"
#include "helper_functions.h"
#include "visual.hpp"


void DrawAngularVelocity(const RigidBody& body) {
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    
    // Начальная точка - центр масс тела
    GLfloat start[3] = {
        static_cast<GLfloat>(body.x[0]),
        static_cast<GLfloat>(body.x[1]),
        static_cast<GLfloat>(body.x[2])
    };
    
    // Конечная точка - центр масс + omega (масштабируем для наглядности)
    const float scale = 7.0f; // Масштабный коэффициент для визуализации
    GLfloat end[3] = {
        start[0] + static_cast<GLfloat>(body.omega[0]) * scale,
        start[1] + static_cast<GLfloat>(body.omega[1]) * scale,
        start[2] + static_cast<GLfloat>(body.omega[2]) * scale
    };
    
    // Рисуем вектор
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.8f, 0.0f); // Жёлто-оранжевый цвет
    glVertex3fv(start);
    glVertex3fv(end);
    
    // Рисуем стрелочку (конус)
    float arrow_size = 1.f;
    glVertex3fv(end);
    glVertex3f(end[0] - body.omega[0]*arrow_size + body.omega[1]*arrow_size*0.3f,
              end[1] - body.omega[1]*arrow_size - body.omega[0]*arrow_size*0.3f,
              end[2] - body.omega[2]*arrow_size);
    
    glVertex3fv(end);
    glVertex3f(end[0] - body.omega[0]*arrow_size - body.omega[1]*arrow_size*0.3f,
              end[1] - body.omega[1]*arrow_size + body.omega[0]*arrow_size*0.3f,
              end[2] - body.omega[2]*arrow_size);
    glEnd();
    
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void DrawTetrahedron(const RigidBody& body) 
{
    glPushMatrix();
    glTranslatef(body.x[0], body.x[1], body.x[2]);

    GLfloat rotation[16] = {
        static_cast<GLfloat>(body.R[0]), static_cast<GLfloat>(body.R[3]), static_cast<GLfloat>(body.R[6]), 0,
        static_cast<GLfloat>(body.R[1]), static_cast<GLfloat>(body.R[4]), static_cast<GLfloat>(body.R[7]), 0,
        static_cast<GLfloat>(body.R[2]), static_cast<GLfloat>(body.R[5]), static_cast<GLfloat>(body.R[8]), 0,
        0, 0, 0, 1
    };

    glMultMatrixf(rotation);

    GLfloat color[] = {0.6f, 0.2f, 0.1f, 1.0f};
    GLfloat ambcolor[] = {0.6f, 0.2f, 0.1f, 1.0f};
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambcolor);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, color);

    GLfloat a[3], b[3], c[3], d[3];
    
    for (int i = 0; i < 3; ++i) {
        a[i] = static_cast<GLfloat>(body.a_vertex[i]);
        b[i] = static_cast<GLfloat>(body.b_vertex[i]);
        c[i] = static_cast<GLfloat>(body.c_vertex[i]);
        d[i] = static_cast<GLfloat>(body.d_vertex[i]);
    }

    glBegin(GL_TRIANGLES);

    // Грани тетраэдра
    // Грань 1: a, b, c (красный цвет)
    glColor3f(1.0f, 0.0f, 0.0f); // Красный
    glVertex3fv(a);
    glVertex3fv(b);
    glVertex3fv(c);

    // Грань 2: a, b, d (зелёный цвет)
    glColor3f(0.0f, 1.0f, 0.0f); // Зелёный
    glVertex3fv(a);
    glVertex3fv(b);
    glVertex3fv(d);

    // Грань 3: a, c, d (синий цвет)
    glColor3f(0.0f, 0.0f, 1.0f); // Синий
    glVertex3fv(a);
    glVertex3fv(c);
    glVertex3fv(d);

    // Грань 4: b, c, d (жёлтый цвет)
    glColor3f(1.0f, 1.0f, 0.0f); // Жёлтый
    glVertex3fv(b);
    glVertex3fv(c);
    glVertex3fv(d);

    glEnd();

    glPopMatrix();

    //DrawAngularVelocity(body);
}


void DrawPlane(const RigidBody& body) 
{
    glPushMatrix();
    
    // Размер плоскости
    const float size = 10.0f;
    // Количество клеток
    const int divisions = 10;
    // Размер одной клетки
    const float step = size / divisions;
    
    // Основной цвет плоскости (серый)
    GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};
    
    // Рисуем разметку (белые линии)
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3fv(white);
    
    // Горизонтальные линии
    for(int i = 0; i <= divisions; ++i) {
        float y = -size/2 + i * step;
        glVertex3f(-size/2, y, 0); // Немного приподнимаем линии над плоскостью
        glVertex3f(size/2, y, 0);
    }
    
    // Вертикальные линии
    for(int i = 0; i <= divisions; ++i) {
        float x = -size/2 + i * step;
        glVertex3f(x, -size/2, 0);
        glVertex3f(x, size/2, 0);
    }
    
    glEnd();
    glEnable(GL_LIGHTING);
    
    glPopMatrix();
}

void DrawAxes() 
{
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0.2f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(5.0f, 0.0f, 0.0f);

    glColor3f(0.2f, 1, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 5.0f, 0.0f);

    glColor3f(0.2f, 0.2f, 1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 5.0f);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void initGL() 
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}