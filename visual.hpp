#pragma once

#define WINDOW_HEIGHT 640
#define WINDOW_WIDTH 640

#include "struct.h"

// Отрисовка тел ПОКА ТЕТРАЭДР
void DrawTetrahedron(const RigidBody& body);

void DrawPlane(const RigidBody& body); 

// Отрисовка осей
void DrawAxes();

// Настройка opengl
void initGL();