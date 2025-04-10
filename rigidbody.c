#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "struct.h"
#include "helper_functions.h"
#include "rigitbody.h"

/* Copy the state information into an array */
void StateToArray(RigidBody *rb, double *y)
{
    *y++ = rb->x[0]; /* x component of position */
    *y++ = rb->x[1]; /* etc. */
    *y++ = rb->x[2];

    *y++ = rb->q.s;
    *y++ = rb->q.v[0];
    *y++ = rb->q.v[1];
    *y++ = rb->q.v[2];

    *y++ = rb->P[0];
    *y++ = rb->P[1]; 
    *y++ = rb->P[2];

    *y++ = rb->L[0];
    *y++ = rb->L[1];
    *y++ = rb->L[2];
}

/* Copy information from an array into the state variables */
void ArrayToState(RigidBody *rb, double *y)
{
    rb->x[0] = *y++;
    rb->x[1] = *y++;
    rb->x[2] = *y++;

    rb->q.s = *y++;
    rb->q.v[0] = *y++;
    rb->q.v[1] = *y++;
    rb->q.v[2] = *y++;

    rb->P[0] = *y++;
    rb->P[1] = *y++;
    rb->P[2] = *y++;

    rb->L[0] = *y++;
    rb->L[1] = *y++;
    rb->L[2] = *y++;

    /* Compute auxiliary variables... */


    quaternion_normalize(&rb->q);  // Нормализуем кватернион

    quaternionToMatrix(rb->q, rb->R);  // Считаем матрицу поворота

    /* v(t) = P(t) M */
    for (int i=0; i<3; i++)
        rb->v[i] = rb->P[i] / rb->mass;

    /* I−1(t) = R(t)I−1 bodyR(t)T*/

    double temp[9];
    matrix_3x3_multiply(rb->R, rb->Ibodyinv, temp);
    double R_transpose[9];
    matrix_3x3_transpose(rb->R, R_transpose);
    matrix_3x3_multiply(temp, R_transpose, rb->Iinv);

    /* ω(t) = I−1(t)L(t) */
    matrix_triple_multiply(rb->Iinv, rb->L, rb->omega);
}


void ArrayToBodies(double x[], RigidBody* Bodies, int NBODIES)
{
    for(int i = 0; i < NBODIES; i++)
        ArrayToState(&Bodies[i], &x[i * STATE_SIZE]);
}


void BodiesToArray(double x[], RigidBody* Bodies, int NBODIES)
{
    for(int i = 0; i < NBODIES; i++)
        StateToArray(&Bodies[i], &x[i * STATE_SIZE]);
}


void ComputeForceAndTorque(double t, RigidBody *rb) // записываем какие силы действуют в момент времени t, пока силы - константы
{
    //rb->force[0] = 0.0;
    //rb->force[1] = 0.;
    //rb->force[2] = -0.000001;

    //rb->torque[0] = 0.0;
    //rb->torque[1] = 0.0;
    //rb->torque[2] = 0.0;
}

void DdtStateToArray(RigidBody *rb, double *xdot)
{
    /* copy d/dt x(t) = v(t) into xdot */
    *xdot++ = rb->v[0];
    *xdot++ = rb->v[1];
    *xdot++ = rb->v[2];

    quaternion qdot;

    qdot.s = 0;
    qdot.v[0] = 0;
    qdot.v[1] = 0;
    qdot.v[2] = 0;


    quaternion omega_quat;

    omega_quat.s = 0;

    omega_quat.v[0] = rb->omega[0];
    omega_quat.v[1] = rb->omega[1];
    omega_quat.v[2] = rb->omega[2];

    quaternion_multiplication(rb->q, omega_quat, &qdot);   // dq/dt = 0.5*omega*q

    qdot.s *= 0.5;
    qdot.v[0] *= 0.5;
    qdot.v[1] *= 0.5;
    qdot.v[2] *= 0.5;
    
    *xdot++ = qdot.s;
    *xdot++ = qdot.v[0];
    *xdot++ = qdot.v[1];
    *xdot++ = qdot.v[2];

    *xdot++ = rb->force[0]; /* d/dt P(t) = F(t) */
    *xdot++ = rb->force[1];
    *xdot++ = rb->force[2];

    *xdot++ = rb->torque[0]; /* ddt L(t) = τ(t) */
    *xdot++ = rb->torque[1];
    *xdot++ = rb->torque[2];
}

void Dxdt(double t, double x[], double xdot[], void* data)
{
    // Приводим data к типу SimulationData*
    struct SimulationData* simData = (struct SimulationData*)data;

    // Извлекаем Bodies и NBODIES
    RigidBody* Bodies = simData->Bodies;
    int NBODIES = simData->NBODIES;

    /* put data in x[] into Bodies[] */
    ArrayToBodies(x, Bodies, NBODIES);
    for(int i = 0; i < NBODIES; i++)
    {
        ComputeForceAndTorque(t, &Bodies[i]);
        DdtStateToArray(&Bodies[i], &xdot[i * STATE_SIZE]);
    }
}

void InitTetrahedron(RigidBody* Body)
{
    // Вершины неправильного тетраэдра
    //double a[3] = {1, 0, 1.1};
    //double b[3] = {0, 1, 1.1};
    //double c[3] = {1, 1, 1.1};
    //double d[3] = {0.6, 0.6, 1.9};

    // С этими данными можно сравнить пример расчета тензора инерции из статьи
    //double a[3] = {8.3322, -11.86875, 0.93355};
    //double b[3] = {0.75523, 5., 16.37072};
    //double c[3] = {52.61236, 5., -5.38580};
    //double d[3] = {2., 5., 3.}; 

    //platonic tetrahedron

    double a[3] = {1, 0, -1/sqrt(2)};
    double b[3] = {-1, 0, -1/sqrt(2)};
    double c[3] = {0, 1, 1/sqrt(2)};
    double d[3] = {0, -1, 1/sqrt(2)};

    a[2] += 6;
    b[2] += 6;
    c[2] += 6;
    d[2] += 6;

    for (int i=0; i<3; i++)
    {
        a[i] /= 6;
        b[i] /= 6;
        c[i] /= 6;
        d[i] /= 6;
    }

    Body->x[0] = (a[0] + b[0] + c[0] + d[0]) / 4;  // Расчет координат центра масс
    Body->x[1] = (a[1] + b[1] + c[1] + d[1]) / 4;  // в системе координат
    Body->x[2] = (a[2] + b[2] + c[2] + d[2]) / 4;  // связанной с телом

    for(int i=0; i<3; i++)
    {
        Body->a_vertex[i] = a[i] - Body->x[i];
        Body->b_vertex[i] = b[i] - Body->x[i]; // Расчет координат
        Body->c_vertex[i] = c[i] - Body->x[i]; // вершин в системе координат
        Body->d_vertex[i] = d[i] - Body->x[i]; // связанной с телом
    }

    double density = 1;  // Плотность

    double R[9];
    compute_R(Body->a_vertex, Body->b_vertex,
    Body->c_vertex, Body->d_vertex, R);
    Body->mass = compute_mass_tetrahedron(density, R);

    calculateTetrahedronInertia(Body->a_vertex, Body->b_vertex,
    Body->c_vertex, Body->d_vertex, density, Body->Ibody);  // Расчет тензора инерции
    
    matrix_3x3_inverse(Body->Ibody, Body->Ibodyinv);  // Расчет инвертированного тензора инерции


    // Начальные значения кватерниона
    Body->q.s = 1;

    Body->q.v[0] = 0;
    Body->q.v[1] = 0;
    Body->q.v[2] = 0;

    // Начальные значения момента силы
    //Body->L[0] = 0.00001;
    //Body->L[1] = 0.000001;
    //Body->L[2] = 0.0001;


    //Body->P[2] = -0.001;

    Body->force[2] = -Body->mass*1/5;
}

void InitPlane(RigidBody* Body)
{
    Body->mass = __INT_MAX__;
    Body->x[0] = 0;
    Body->x[1] = 0;
    Body->x[2] = 0;

    for (int i=0; i<9; i++)
        Body->Ibodyinv[i] = 0;

    Body->q.s = 1;

    Body->q.v[0] = 0;
    Body->q.v[1] = 0;
    Body->q.v[2] = 0;
}

void InitStates(RigidBody* Bodies)
{
    InitTetrahedron(&Bodies[0]);

    InitPlane(&Bodies[1]);
}