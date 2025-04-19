#include <stdio.h>
#include <stdlib.h>


#include "struct.h"
#include "helper_functions.h"


int Ncontacts;
Contact *Contacts;

int check_vertex_collision(RigidBody* Body, char vertex)
{
    double temp_a[3], temp_b[3], temp_c[3], temp_d[3];

    matrix_triple_multiply(Body->R, Body->a_vertex, temp_a); // Перевод коориднат
    matrix_triple_multiply(Body->R, Body->b_vertex, temp_b); // вершин из системы,
    matrix_triple_multiply(Body->R, Body->c_vertex, temp_c); // жеско связанной с телом,
    matrix_triple_multiply(Body->R, Body->d_vertex, temp_d); // в повернутую систему тела.

    switch (vertex)
    {
    case 'a':
        if (temp_a[2] + Body->x[2] <= 0)  // Перевод в мировую систему
            return 1;
        break;
    case 'b':
        if (temp_b[2] + Body->x[2] <= 0)
            return 1;
        break;
    case 'c':
        if (temp_c[2] + Body->x[2]<= 0)
            return 1;
        break;
    case 'd':
        if (temp_d[2] + Body->x[2] <= 0)
            return 1;
        break;
    default:
        return 0;
        break;
    }

    return 0;
    
}

/* Return the velocity of a point on a rigid body */
void pt_velocity(RigidBody *body, double* p, double res[3])
{
    double temp_sub[3];
    vector_subtract(p, body->x, temp_sub);
    double temp_mult[3];
    vector_miltiplication(body->omega, temp_sub, temp_mult);
    vector_add(body->v, temp_mult, res);
}

/*
* Return true if bodies are in colliding contact. The
* parameter ‘THRESHOLD’ is a small numerical tolerance
* used for deciding if bodies are colliding.
*/
int colliding(Contact *c)
{

    double t = 0.0000001;

    double padot[3], pbdot[3];
    pt_velocity(c->a, c->p, padot); /* p˙− a (t0 ) */
    pt_velocity(c->b, c->p, pbdot); /* p˙− b (t0 ) */
    
    double p_diff[3];
    vector_subtract(padot, pbdot, p_diff);

    double vrel = scalar_multiplication(c->n, p_diff);
    
    if(vrel > t) /* moving away */
        return 0;
    if(vrel > -t) /* resting contact */
        return 0;
    else /* vrel < -THRESHOLD */
        return 1;
}



void collision(Contact *c, double epsilon)
{
    double padot[3], pbdot[3];
    pt_velocity(c->a, c->p, padot); /* p˙− a (t0) */
    pt_velocity(c->b, c->p, pbdot); /* p˙−b (t0) */

    double n[3];
    for (int i=0; i<3; i++) n[i] = c->n[i];

    double ra[3]; /* ra */
    double rb[3]; /* rb */

    vector_subtract(c->p, c->a->x, ra);
    vector_subtract(c->p, c->b->x, rb);

    double p_diff[3];

    vector_subtract(padot, pbdot, p_diff);

    double vrel = scalar_multiplication(n, p_diff);

    double numerator = -(1 + epsilon) * vrel;

    /* We’ll calculate the denominator in four parts */

    double term1 = 1 / c->a->mass;
    double term2 = 1 / c->b->mass;

    double ra_cross_n[3], rb_cross_n[3];
    double Iinv_mult_ra_cross_n[3], Iinv_mult_rb_cross_n[3];
    double Iinv_mult_ra_cross_n_cross_ra[3], Iinv_mult_rb_cross_n_cross_rb[3];

    vector_miltiplication(ra, n, ra_cross_n);
    vector_miltiplication(rb, n, rb_cross_n);

    matrix_triple_multiply(c->a->Iinv, ra_cross_n, Iinv_mult_ra_cross_n);
    matrix_triple_multiply(c->b->Iinv, rb_cross_n, Iinv_mult_rb_cross_n);

    vector_miltiplication(Iinv_mult_ra_cross_n, ra, Iinv_mult_ra_cross_n_cross_ra);
    vector_miltiplication(Iinv_mult_rb_cross_n, rb, Iinv_mult_rb_cross_n_cross_rb);

    double term3 = scalar_multiplication(n, Iinv_mult_ra_cross_n_cross_ra);
    double term4 = scalar_multiplication(n, Iinv_mult_rb_cross_n_cross_rb);


    /* Compute the impulse magnitude */

    double j = numerator / (term1 + term2 + term3 + term4);

    double force[3];

    scalar_vector_multiply(j, n, force);

    /* Apply the impulse to the bodies */

    double ra_cross_force[3], rb_cross_force[3];

    vector_miltiplication(ra, force, ra_cross_force);
    vector_miltiplication(rb, force, rb_cross_force); 
    
    for (int i=0; i<3; i++)
    {
        c->a->P[i] += force[i];
        c->b->P[i] -= force[i];

        c->a->L[i] += ra_cross_force[i];
        c->b->L[i] -= rb_cross_force[i];

        /* recompute auxiliary variables */
        c->a->v[i] = c->a->P[i] / c->a->mass;
        c->b->v[i] = c->b->P[i] / c->b->mass;
    }

    matrix_triple_multiply(c->a->Iinv, c->a->L, c->a->omega);
    matrix_triple_multiply(c->b->Iinv, c->b->L, c->b->omega);

}

void compute_collision(RigidBody* Bodies, char vertex)
{

    Contact c;  // Структура, хранящая данные о столкновении
    c.a = &Bodies[0];  // Первое тело из пары
    c.b = &Bodies[1];  // Второе тело из пары

    c.n[0] = 0;  // Вектор нормали к плоскости столкновения
    c.n[1] = 0;  // в рассматриваемой задаче является констнтой
    c.n[2] = 1;  // нормаль к плоскости z = 0 (0;0;1)

    double temp[3];  // Временная переменная для расчета координат вершин в мировой системе

    switch (vertex)
    {
    case 'a':
        matrix_triple_multiply(Bodies[0].R, Bodies[0].a_vertex, temp); // Перевод в мировую систему
        vector_add(temp, Bodies[0].x, c.p);
        break;
    case 'b':
        matrix_triple_multiply(Bodies[0].R, Bodies[0].b_vertex, temp);
        vector_add(temp, Bodies[0].x, c.p);
        break;
    case 'c':
        matrix_triple_multiply(Bodies[0].R, Bodies[0].c_vertex, temp);
        vector_add(temp, Bodies[0].x, c.p);
        break;
    case 'd':
        matrix_triple_multiply(Bodies[0].R, Bodies[0].d_vertex, temp);
        vector_add(temp, Bodies[0].x, c.p);
        break;
    }
    
    if ( colliding(&c) )
        collision(&c, 1);  // 1 - абсолютно упругий отскок, 0 - абсолютно неупругий.

}


void check_and_compute_collision(RigidBody* Bodies)
{

    if (check_vertex_collision(&Bodies[0], 'a'))
    {
        compute_collision(Bodies, 'a');
    }
    if (check_vertex_collision(&Bodies[0], 'b'))
    {
        compute_collision(Bodies, 'b');
    }
    if (check_vertex_collision(&Bodies[0], 'c'))
    {
        compute_collision(Bodies, 'c');
    }
    if (check_vertex_collision(&Bodies[0], 'd'))
    {
        compute_collision(Bodies, 'd');
    }
}