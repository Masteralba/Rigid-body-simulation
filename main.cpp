#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>

#include "helper_functions.h"
#include "struct.h"
#include "rigitbody.h"
#include "collision.h"
#include "visual.hpp"

#define NBODIES 2 // Количество тел в симуляции

RigidBody Bodies[NBODIES]; // Глобальный массив тел

double simulationTime = 0; // Глобальная переменная времени

Rk4 rk4 = {NULL, NULL, NULL, NULL, NULL};  // Глобальная переменная структуры РК-4

void RenderScene() // Находится здесь, потому что требует глобальную переменную Bodies
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (double)WINDOW_WIDTH / WINDOW_HEIGHT, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(0.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    GLfloat light_position[] = {5.0, 5.0, 5.0, 1.0};
    GLfloat light_color[] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);

    glEnable(GL_LIGHTING);
    DrawTetrahedron(Bodies[0]);  // Использование глобальной переменной
    DrawPlane(Bodies[1]);
    glEnable(GL_LIGHTING);

    glutSwapBuffers();
}

void update(int value) {
    
    glutPostRedisplay();  // вызывает функцию RenderScene

    double x0[STATE_SIZE * NBODIES], // Массив векторов состояний до шага моделирования
    xFinal[STATE_SIZE * NBODIES];   // Массив векторов состояний после шага моделирования

    check_and_compute_collision(Bodies);

    BodiesToArray(x0, Bodies, NBODIES);  // Представляем тела в виде вектора состояния

    double dt = 1.0 / 30.0;   // Шаг симуляции

    struct SimulationData data = {Bodies, NBODIES};  // Дополнительные данные для ode

    ode(&rk4, x0, xFinal, STATE_SIZE * NBODIES, simulationTime, simulationTime+dt, Dxdt, &data); // Выполняем шаг симуляции

    ArrayToBodies(xFinal, Bodies, NBODIES); // Записываем новый вектор состояния в тела

    simulationTime += dt;  // Увеличиваем время симуляции

    // Планирование следующего вызова update через 16 мс (примерно 60 кадров в секунду)
    glutTimerFunc(16, update, 0);
}


int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Rigid Body Simulation");

    initGL();

    InitStates(Bodies);

    glutDisplayFunc(RenderScene);

    glutTimerFunc(0, update, 0);

    glutMainLoop();

    rk4Free(&rk4);  // освобождение памяти из-под РК-4
 }

