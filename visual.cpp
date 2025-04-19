#include <GL/glut.h>

#include "struct.h"
#include "helper_functions.h"
#include "visual.hpp"


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

    // Желто-оранжевый материал
    GLfloat orange_diffuse[] = {1.0f, 0.6f, 0.2f, 1.0f};
    GLfloat orange_ambient[] = {0.3f, 0.2f, 0.1f, 1.0f};
    GLfloat orange_specular[] = {1.0f, 1.0f, 0.8f, 1.0f};
    GLfloat shininess = 50.0f;

    glMaterialfv(GL_FRONT, GL_AMBIENT, orange_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, orange_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, orange_specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);

    GLfloat a[3], b[3], c[3], d[3];
    
    for (int i = 0; i < 3; ++i) {
        a[i] = static_cast<GLfloat>(body.a_vertex[i]);
        b[i] = static_cast<GLfloat>(body.b_vertex[i]);
        c[i] = static_cast<GLfloat>(body.c_vertex[i]);
        d[i] = static_cast<GLfloat>(body.d_vertex[i]);
    }

    glBegin(GL_TRIANGLES);
    // Все грани одного цвета
    // Грань 1: a, b, c
    glNormal3f(0, 0, 1); // Простая нормаль (можно улучшить)
    glVertex3fv(a);
    glVertex3fv(b);
    glVertex3fv(c);

    // Грань 2: a, b, d
    glNormal3f(0, 1, 0);
    glVertex3fv(a);
    glVertex3fv(b);
    glVertex3fv(d);

    // Грань 3: a, c, d
    glNormal3f(1, 0, 0);
    glVertex3fv(a);
    glVertex3fv(c);
    glVertex3fv(d);

    // Грань 4: b, c, d
    glNormal3f(0, 0, -1);
    glVertex3fv(b);
    glVertex3fv(c);
    glVertex3fv(d);
    glEnd();

    glPopMatrix();

}


void DrawPlane(const RigidBody& body) 
{
    glPushMatrix();
    
    // Размер плоскости
    const float size = 2.5f;
    // Количество клеток
    const int divisions = 8;
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

void initGL() 
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    
    // Настройка света
    GLfloat light_position[] = {.0f, 0.0f, .0f, 0.0f};
    GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat light_ambient[] = {1.f, 1.f, 1.6f, 2.0f};
    GLfloat light_specular[] = {3.0f, 3.0f, 3.0f, 3.0f};
    
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glEnable(GL_LIGHT0);
    
}