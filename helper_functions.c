#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "struct.h"

void matrix_add(double* A, double* B, double* result) {
    for (int i = 0; i < 9; i++) {
        result[i] = A[i] + B[i];
    }
}

void matrix_subtract(double* A, double* B, double* result) {
    for (int i = 0; i < 9; i++) {
        result[i] = A[i] - B[i];
    }
}

void vector_add(double* a, double* b, double* res)
{
    for (int i=0; i<3; i++) res[i] = a[i] +b[i];
}

void vector_subtract(double* a, double* b, double* res)
{
    for (int i=0; i<3; i++) res[i] = a[i] - b[i];
}

void scalar_vector_multiply(double lambda, double* a, double* res)
{
    for (int i=0; i<3; i++)
        res[i] = lambda*a[i];
}

void matrix_3x3_multiply(double* A, double* B, double* result) {
    for (int i = 0; i < 3; i++) {         // Строки матрицы A
        for (int j = 0; j < 3; j++) {     // Столбцы матрицы B
            result[i * 3 + j] = 0;        // Инициализация элемента результата
            for (int k = 0; k < 3; k++) { // Сумма произведений
                result[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }
}

void matrix_triple_multiply(double* matrix, double* triple, double* res)
{
    res[0] = matrix[0]*triple[0] + matrix[1]*triple[1] + matrix[2]*triple[2];
    res[1] = matrix[3]*triple[0] + matrix[4]*triple[1] + matrix[5]*triple[2];
    res[2] = matrix[6]*triple[0] + matrix[7]*triple[1] + matrix[8]*triple[2];
}


double determinant_3x3(double* mat) {
    return mat[0] * (mat[4] * mat[8] - mat[5] * mat[7]) -
           mat[1] * (mat[3] * mat[8] - mat[5] * mat[6]) +
           mat[2] * (mat[3] * mat[7] - mat[4] * mat[6]);
}

// Функция для нахождения обратной матрицы
int matrix_3x3_inverse(double* mat, double* result) {
    // Вычисляем определитель
    double det = determinant_3x3(mat);
    if (det == 0) {
        printf("Обратной матрицы не существует (определитель равен нулю).\n");
        return 0; // Возвращаем 0, если обратной матрицы нет
    }

    // Вычисляем союзную матрицу (матрицу алгебраических дополнений)
    result[0] = (mat[4] * mat[8] - mat[5] * mat[7]) / det;
    result[1] = (mat[2] * mat[7] - mat[1] * mat[8]) / det;
    result[2] = (mat[1] * mat[5] - mat[2] * mat[4]) / det;

    result[3] = (mat[5] * mat[6] - mat[3] * mat[8]) / det;
    result[4] = (mat[0] * mat[8] - mat[2] * mat[6]) / det;
    result[5] = (mat[2] * mat[3] - mat[0] * mat[5]) / det;

    result[6] = (mat[3] * mat[7] - mat[4] * mat[6]) / det;
    result[7] = (mat[1] * mat[6] - mat[0] * mat[7]) / det;
    result[8] = (mat[0] * mat[4] - mat[1] * mat[3]) / det;

    return 1; // Успешное завершение
}

void compute_R(double* a, double* b, double* c, double* d, double* R)
{

    R[0] = b[0] - a[0];
    R[1] = c[0] - a[0];
    R[2] = d[0] - a[0];

    R[3] = b[1] - a[1];
    R[4] = c[1] - a[1];
    R[5] = d[1] - a[1];

    R[6] = b[2] - a[2];
    R[7] = c[2] - a[2];
    R[8] = d[2] - a[2];

}

void print_3x3_matrix(double* matrix) {
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            printf("%f ", matrix[row * 3 + col]);
        }
        printf("\n");
    }
}

double compute_mass_tetrahedron(double density, double* R)
{
    return density*abs(determinant_3x3(R))/6;
}

void matrix_3x3_transpose(double* matrix, double* res)
{   
    res[0] = matrix[0];
    res[1] = matrix[3];
    res[2] = matrix[6];

    res[3] = matrix[1];
    res[4] = matrix[4];
    res[5] = matrix[7];

    res[6] = matrix[2];
    res[7] = matrix[5];
    res[8] = matrix[8];
}

void ode(Rk4* self, double* x, double* xFinal, int n, double t0, double t1, void (*f)(double, double*, double*, void*), void* data)
{

    if (self->k1 == NULL)
        self->k1 = (double*)malloc(sizeof(double)*n);

    if (self->k2 == NULL)
        self->k2 = (double*)malloc(sizeof(double)*n);

    if (self->k3 == NULL)
        self->k3 = (double*)malloc(sizeof(double)*n);

    if (self->k4 == NULL)
        self->k4 = (double*)malloc(sizeof(double)*n);

    if (self->tmp == NULL)
        self->tmp = (double*)malloc(sizeof(double)*n);

    double h = t1-t0;

    f(t0, x, self->k1, data); // now k1 has the right part of f(t, x)

    for (int i=0; i<n; i++)
        self->tmp[i] = x[i] + h*0.5*self->k1[i]; // now tmp is x_n + h/2*k1

    f(t0+h*0.5, self->tmp, self->k2, data);  // now k2 has the right part f(t/2, x_n+h/2*k1)

    for (int i=0; i<n; i++)
        self->tmp[i] = x[i] + h*0.5*self->k2[i];  // now tmp is x_n + h/2*k2

    f(t0+h*0.5, self->tmp, self->k3, data);  // now k3 has the right part f(t/2, x_n+h/2*k2)

    for (int i=0; i<n; i++)
        self->tmp[i] = x[i] + h*self->k3[i]; // now tmp is x_n + h*k3

    f(t0, self->tmp, self->k4, data);  // now fx has the right part f(k3)

    for (int i=0; i<n; i++)
        xFinal[i] = x[i] + 1./6.*(self->k1[i]+2*self->k2[i]+2*self->k3[i]+self->k4[i]);

}

void rk4Free(Rk4* rk)
{
    free(rk->k1);
    free(rk->k2);
    free(rk->k3);
    free(rk->k4);
    free(rk->tmp);
}

void normalize(double* v) {
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm > 0.0) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

void orthogonalize_matrix(double matrix[9]) {
    double *v1 = &matrix[0];
    double *v2 = &matrix[3];
    double *v3 = &matrix[6];
    
    // Нормализация первого вектора
    normalize(v1);
    
    // Ортогонализация второго вектора относительно первого
    double dot = v2[0]*v1[0] + v2[1]*v1[1] + v2[2]*v1[2];
    v2[0] -= dot * v1[0];
    v2[1] -= dot * v1[1];
    v2[2] -= dot * v1[2];
    normalize(v2);
    
    // Третий вектор как векторное произведение первых двух (чтобы обеспечить правую ориентацию)
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    // Не нужно нормализовать, так как v1 и v2 уже ортонормированы
    
    // Проверка ориентации (чтобы определитель был +1)
    double det = v1[0]*(v2[1]*v3[2] - v2[2]*v3[1]) -
                 v1[1]*(v2[0]*v3[2] - v2[2]*v3[0]) +
                 v1[2]*(v2[0]*v3[1] - v2[1]*v3[0]);
    
    if (det < 0) {
        // Если ориентация отрицательная, инвертируем третий вектор
        v3[0] = -v3[0];
        v3[1] = -v3[1];
        v3[2] = -v3[2];
    }
}


double scalar_multiplication(double* a, double* b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vector_miltiplication(double* a, double* b, double* res)
{
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0]; 
}

void quaternion_multiplication(quaternion a, quaternion b, quaternion* res)
{
    res->s = a.s*b.s - scalar_multiplication(a.v, b.v);

    res->v[0] = a.s*b.v[0] + b.s*a.v[0] + a.v[1]*b.v[2] - a.v[2]*b.v[1];
    res->v[1] = a.s*b.v[1] + b.s*a.v[1] + a.v[2]*b.v[0] - a.v[0]*b.v[2];
    res->v[2] = a.s*b.v[2] + b.s*a.v[2] + a.v[0]*b.v[1] - a.v[1]*b.v[0];
}

void quaternion_normalize(quaternion* q)
{
    double l = sqrt(q->v[0]*q->v[0] + q->v[1]*q->v[1] + q->v[2]*q->v[2] + q->s*q->s);

    q->s = q->s / l;
    q->v[0] = q->v[0] / l;
    q->v[1] = q->v[1] / l;
    q->v[2] = q->v[2] / l; 
}

void quaternionToMatrix(quaternion q, double* res)
{
    res[0] = 1 - 2*q.v[1]*q.v[1] - 2*q.v[2]*q.v[2];
    res[1] = 2*q.v[0]*q.v[1] - 2*q.s*q.v[2];
    res[2] = 2*q.v[0]*q.v[2] + 2*q.s*q.v[1];

    res[3] = 2*q.v[0]*q.v[1] + 2*q.s*q.v[2];
    res[4] = 1-2*q.v[0]*q.v[0]-2*q.v[2]*q.v[2];
    res[5] = 2*q.v[1]*q.v[2] - 2*q.s*q.v[0];
    
    res[6] = 2*q.v[0]*q.v[2] - 2*q.s*q.v[1];
    res[7] = 2*q.v[1]*q.v[2] + 2*q.s*q.v[0];
    res[8] = 1-2*q.v[0]*q.v[0]-2*q.v[1]*q.v[1];
}


void calculateTetrahedronInertia(double* a_v, double* b_v, double* c_v, double* d_v, double density, double inertia_tensor[9]) {
    // Извлекаем координаты вершин
    double x1 = a_v[0], y1 = a_v[1], z1 = a_v[2];
    double x2 = b_v[0], y2 = b_v[1], z2 = b_v[2];
    double x3 = c_v[0], y3 = c_v[1], z3 = c_v[2];
    double x4 = d_v[0], y4 = d_v[1], z4 = d_v[2];
    
    // Вычисляем элементы матрицы Якоби
    double j11 = x2 - x1, j12 = x3 - x1, j13 = x4 - x1;
    double j21 = y2 - y1, j22 = y3 - y1, j23 = y4 - y1;
    double j31 = z2 - z1, j32 = z3 - z1, j33 = z4 - z1;
    
    // Вычисляем определитель матрицы Якоби
    double detJ = j11 * (j22 * j33 - j23 * j32) 
                - j12 * (j21 * j33 - j23 * j31) 
                + j13 * (j21 * j32 - j22 * j31);
    
    double absDetJ = fabs(detJ);
    double mu_detJ = density * absDetJ;
    
    // Вычисляем элементы тензора инерции
    double a = mu_detJ * (
        y1*y1 + y1*y2 + y2*y2 + y1*y3 + y2*y3 + y3*y3 + 
        y1*y4 + y2*y4 + y3*y4 + y4*y4 +
        z1*z1 + z1*z2 + z2*z2 + z1*z3 + z2*z3 + z3*z3 + 
        z1*z4 + z2*z4 + z3*z4 + z4*z4
    ) / 60.0;
    
    double b = mu_detJ * (
        x1*x1 + x1*x2 + x2*x2 + x1*x3 + x2*x3 + x3*x3 + 
        x1*x4 + x2*x4 + x3*x4 + x4*x4 +
        z1*z1 + z1*z2 + z2*z2 + z1*z3 + z2*z3 + z3*z3 + 
        z1*z4 + z2*z4 + z3*z4 + z4*z4
    ) / 60.0;
    
    double c = mu_detJ * (
        x1*x1 + x1*x2 + x2*x2 + x1*x3 + x2*x3 + x3*x3 + 
        x1*x4 + x2*x4 + x3*x4 + x4*x4 +
        y1*y1 + y1*y2 + y2*y2 + y1*y3 + y2*y3 + y3*y3 + 
        y1*y4 + y2*y4 + y3*y4 + y4*y4
    ) / 60.0;
    
    double ap = mu_detJ * (
        2*y1*z1 + y2*z1 + y3*z1 + y4*z1 + 
        y1*z2 + 2*y2*z2 + y3*z2 + y4*z2 + 
        y1*z3 + y2*z3 + 2*y3*z3 + y4*z3 + 
        y1*z4 + y2*z4 + y3*z4 + 2*y4*z4
    ) / 120.0;
    
    double bp = mu_detJ * (
        2*x1*z1 + x2*z1 + x3*z1 + x4*z1 + 
        x1*z2 + 2*x2*z2 + x3*z2 + x4*z2 + 
        x1*z3 + x2*z3 + 2*x3*z3 + x4*z3 + 
        x1*z4 + x2*z4 + x3*z4 + 2*x4*z4
    ) / 120.0;
    
    double cp = mu_detJ * (
        2*x1*y1 + x2*y1 + x3*y1 + x4*y1 + 
        x1*y2 + 2*x2*y2 + x3*y2 + x4*y2 + 
        x1*y3 + x2*y3 + 2*x3*y3 + x4*y3 + 
        x1*y4 + x2*y4 + x3*y4 + 2*x4*y4
    ) / 120.0;
    
    // Заполняем матрицу 3x3 (по строкам)
    inertia_tensor[0] = a;    // [0][0]
    inertia_tensor[1] = -bp;  // [0][1]
    inertia_tensor[2] = -cp;  // [0][2]
    
    inertia_tensor[3] = -bp;  // [1][0]
    inertia_tensor[4] = b;    // [1][1]
    inertia_tensor[5] = -ap;  // [1][2]
    
    inertia_tensor[6] = -cp;  // [2][0]
    inertia_tensor[7] = -ap;  // [2][1]
    inertia_tensor[8] = c;    // [2][2]
}