#pragma once

#include "struct.h"

char check_collision(RigidBody* Bodies);

void pt_velocity(RigidBody *body, double* p, double res[3]);

int colliding(Contact *c);

void collision(Contact *c, double epsilon);

void FindAllCollisions(Contact contacts[], int ncontacts);