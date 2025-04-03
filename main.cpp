#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>

#include "help_functions.h"
#include "struct.h"
#include "rigitbody.h"
#include "visual.hpp"

#define NBODIES 1 // Количество тел в симуляции

RigidBody Bodies[NBODIES]; // Глобальный массив тел

double simulationTime = 0; // Глобальная переменная времени

void RenderScene() // Находится здесь, потому что требует глобальную переменную Bodies
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (double)WINDOW_WIDTH / WINDOW_HEIGHT, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    GLfloat light_position[] = {5.0, 5.0, 5.0, 1.0};
    GLfloat light_color[] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);

    glDisable(GL_LIGHTING);
    DrawBody(Bodies[0]);  // Использование глобальной переменной
    glEnable(GL_LIGHTING);
    DrawAxes();

    glutSwapBuffers();
}

void update(int value) {

    
    glutPostRedisplay();  // вызывает функцию RenderScene

    double x0[STATE_SIZE * NBODIES], // Массив векторов состояний до шага моделирования
    xFinal[STATE_SIZE * NBODIES];   // Массив векторов состояний после шага моделирования

    BodiesToArray(xFinal, Bodies, NBODIES);

    Rk4 rk4 = {NULL, NULL, NULL, NULL, NULL};

    //for (int j=0; j<STATE_SIZE; j++)
        //printf("%lf ", xFinal[j]);


    double en = scalar_multiplication(Bodies[0].L, Bodies[0].omega)/ 2;
    // Расчет кинетической энергии вращения

    printf("%.15e\n", en);

    // Выполняем один шаг симуляции
    for (int i = 0; i < STATE_SIZE * NBODIES; i++)
        x0[i] = xFinal[i];

    double dt = 1.0 / 24.0;   // Шаг симуляции (1/24 секунды)

    struct SimulationData data = {Bodies, NBODIES};

    ode(&rk4, x0, xFinal, STATE_SIZE * NBODIES, simulationTime, simulationTime+dt, Dxdt, &data);

    ArrayToBodies(xFinal, Bodies, NBODIES);

    rk4Free(&rk4);

    simulationTime += dt;     // Увеличиваем время симуляции

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
 }

