#pragma once

#include "struct.h"

// Загрузка данных вектора состояния твердого тела в массив
void StateToArray(RigidBody *rb, double *y);

// Загрузка данных из массива в вектор состояния твердого тела
void ArrayToState(RigidBody *rb, double *y);

// Загрузка данных из массива в массив тел
void ArrayToBodies(double x[], RigidBody* Bodies, int NBODIES);

// Загрузка данных из массива тел в массив
void BodiesToArray(double x[], RigidBody* Bodies, int NBODIES);

// Силы в момент времени t, ПОКА КОНСТАНТЫ
void ComputeForceAndTorque(double t, RigidBody *rb);

// Загрузка данных из вектора состояния с точкой в массив
void DdtStateToArray(RigidBody *rb, double *xdot);

// Взятие производной у массива тел
void Dxdt(double t, double x[], double xdot[], void* data);

// Установить константные значения тел ПОКА ТЕТРАЭДР
void InitStates(RigidBody* Bodies);



