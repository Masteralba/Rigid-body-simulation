#include <stdio.h>
#include <stdlib.h>

#include "struct.h"
#include "helper_functions.h"


int Ncontacts;
Contact *Contacts;

char check_collision(RigidBody* Bodies)
{
    double temp_a[3], temp_b[3], temp_c[3], temp_d[3];

    matrix_triple_multiply(Bodies[0].R, Bodies[0].a_vertex, temp_a);
    matrix_triple_multiply(Bodies[0].R, Bodies[0].b_vertex, temp_b);
    matrix_triple_multiply(Bodies[0].R, Bodies[0].c_vertex, temp_c);
    matrix_triple_multiply(Bodies[0].R, Bodies[0].d_vertex, temp_d);

    if (temp_a[2] <= 0)
        return 'a';
    if (temp_b[2] <= 0)
        return 'b';
    if (temp_c[2] <= 0)
        return 'c';
    if (temp_d[2]<= 0)
        return 'd';
    
    return '0'; // no collision
}

int check_vertex_collision(RigidBody* Body, char vertex)
{
    double temp_a[3], temp_b[3], temp_c[3], temp_d[3];

    matrix_triple_multiply(Body->R, Body->a_vertex, temp_a);
    matrix_triple_multiply(Body->R, Body->b_vertex, temp_b);
    matrix_triple_multiply(Body->R, Body->c_vertex, temp_c);
    matrix_triple_multiply(Body->R, Body->d_vertex, temp_d);
    switch (vertex)
    {
    case 'a':
        if (temp_a[2] + Body->x[2] <= 0)
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

    double t = 0.00001;

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


void FindAllCollisions(Contact contacts[], int ncontacts)
{
    int had_collision;
    double epsilon = 1;
    do {
        had_collision = 0;
        for(int i = 0; i < ncontacts; i++)
            if(colliding(&contacts[i]))
            {
                collision(&contacts[i], epsilon);
                had_collision = 1;
                /* Tell the solver we had a collision */
                //ode_discontinuous();
            }
    } while(had_collision == 1);
}


void checking(RigidBody* Bodies)
{
    if (check_vertex_collision(&Bodies[0], 'a'))
    {
        double p[3];
        Contact c;
        c.a = &Bodies[0];
        c.b = &Bodies[1];
        c.n[0] = 0;
        c.n[1] = 0;
        c.n[2] = 1;
        vector_add(Bodies[0].a_vertex, Bodies[0].x, p);
        for (int i=0; i< 3; i++)
            c.p[i] = p[i];
        FindAllCollisions(&c, 1);
    }
    if (check_vertex_collision(&Bodies[0], 'b'))
    {
        double p[3];
        Contact c;
        c.a = &Bodies[0];
        c.b = &Bodies[1];
        c.n[0] = 0;
        c.n[1] = 0;
        c.n[2] = 1;
        vector_add(Bodies[0].b_vertex, Bodies[0].x, p);
        for (int i=0; i< 3; i++)
            c.p[i] = p[i];
        FindAllCollisions(&c, 1);
    }
    if (check_vertex_collision(&Bodies[0], 'c'))
    {
        double p[3];
        Contact c;
        c.a = &Bodies[0];
        c.b = &Bodies[1];
        c.n[0] = 0;
        c.n[1] = 0;
        c.n[2] = 1;
        vector_add(Bodies[0].c_vertex, Bodies[0].x, p);
        for (int i=0; i< 3; i++)
            c.p[i] = p[i];
        FindAllCollisions(&c, 1);
    }
    if (check_vertex_collision(&Bodies[0], 'd'))
    {
        double p[3];
        Contact c;
        c.a = &Bodies[0];
        c.b = &Bodies[1];
        c.n[0] = 0;
        c.n[1] = 0;
        c.n[2] = 1;
        vector_add(Bodies[0].d_vertex, Bodies[0].x, p);
        for (int i=0; i< 3; i++)
            c.p[i] = p[i];
        FindAllCollisions(&c, 1);
    }
}