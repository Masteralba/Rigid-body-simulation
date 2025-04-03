#pragma once

#define STATE_SIZE 13  // Размер вектора состояния системы


typedef struct  // Структура кватерниона
{
    double s;  // Скалярная составляющая
    double v[3]; // Векторная составляющая
} quaternion;


typedef struct
{
    /* Constant quantities */
    double mass; /* mass M */
    double Ibody[9]; /* Ibody */
    double Ibodyinv[9]; /* I−1 body (inverse of Ibody) */

    double a_vertex[3];  // vector from center of mass to a-vertex 
    double b_vertex[3];  // vector from center of mass to b-vertex 
    double c_vertex[3];  // vector from center of mass to c-vertex 
    double d_vertex[3];  // vector from center of mass to d-vertex 
        
    /* State variables */
    double x[3]; /* x(t) */  // Координаты центра масс
    quaternion q; /* q(t) */ // Кватернион
    double P[3]; /* P(t) */  // Момент инерции
    double L[3]; /* L(t) */  // Угловой момент

    /* Derived quantities (auxiliary variables) */
    double Iinv[9]; /* I−1(t) */
    double R[9]; /* R(t) */  // Матрица поворота центра масс относительно мировых координат 
    double v[3]; /* v(t) */
    double omega[3]; /* ω(t) */

    /* Computed quantities */
    double force[3]; /* F(t) */
    double torque[3]; /* τ(t) */

}  RigidBody;


typedef struct
{
    double* k1;
    double* k2;
    double* k3;
    double* k4;
    double* tmp;
} Rk4;

struct SimulationData {
    RigidBody* Bodies;
    int NBODIES;
};