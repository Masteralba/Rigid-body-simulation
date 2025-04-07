#pragma once

#include "struct.h"

// Сумма матриц
void matrix_add(double* A, double* B, double* result);

// Разность матриц
void matrix_subtract(double* A, double* B, double* result);

void vector_add(double* a, double* b, double* res);

void vector_subtract(double* a, double* b, double* res);

void scalar_vector_multiply(double lambda, double* a, double* res);

// Умножение матриц 3x3
void matrix_3x3_multiply(double* A, double* B, double* result);

// Умножение 3-мерного вектора на матрицу 3x3
void matrix_triple_multiply(double* matrix, double* triple, double* res);

// Дискриминант матрицы 3x3
double determinant_3x3(double* mat);

// Обратная матрица к матрице 3x3
int matrix_3x3_inverse(double* mat, double* result);

// Нахождение матрицы R (НЕ матрица поворота), нужно для вычисления массы
void compute_R(double* a, double* b, double* c, double* d, double* R);

// Вывод матрицы 3x3
void print_matrix(double* matrix);

// Вычисление массы тетраэдра
double compute_mass_tetrahedron(double density, double* R);

// Вычисление тензора инерции тетраэдра
void calculateTetrahedronInertia(double* a_v, double* b_v, double* c_v, double* d_v, double density, double inertia_tensor[9]);

// Транспонирование матрицы 3x3 
void matrix_3x3_transpose(double* matrix, double* res);

// Численное решение ОДУ (Рунге-Кутта 4)
void ode(Rk4* self, double* x, double* xFinal, int n, double t0, double t1, 
void (*f)(double, double*, double*, void*), void* data); 
// Функция правой части ( время, начальный вектор состояния, конечный вектор состояния, данные)

// Освобождение памяти из под структуры rk4
void rk4Free(Rk4* rk);

// Ортогонализация матрицы
void orthogonalize_matrix(double matrix[9]);

// Скалярное произведение
double scalar_multiplication(double* a, double* b);

// Векторное произведение
void vector_miltiplication(double* a, double* b, double* res);

// Умножение кватернионов
void quaternion_multiplication(quaternion a, quaternion b, quaternion* res);

// Нормализация кватерниона
void quaternion_normalize(quaternion* q);

// Перевод кватерниона в матрицу
void quaternionToMatrix(quaternion q, double* res);