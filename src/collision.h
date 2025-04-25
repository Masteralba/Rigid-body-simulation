#pragma once

#include "struct.h"

void pt_velocity(RigidBody *body, double* p, double res[3]);

int colliding(Contact *c);

void collision(Contact *c, double epsilon);

int check_vertex_collision(RigidBody* Body, char vertex);

void check_and_compute_collision(RigidBody* Bodies);

void compute_collision(RigidBody* Bodies, char vertex);