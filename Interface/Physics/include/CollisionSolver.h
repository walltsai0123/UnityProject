#pragma once

#include "math_headers.h"

class Contact;
class Simulation;

class CollisionSolver
{
public:
    CollisionSolver(Simulation *sim);
    ~CollisionSolver();

    void solve(const VectorX& velocities_p, VectorX& contact, VectorX& friction);
protected:
    Simulation* m_simulation;
    // SoftBody *softbody;
private:
    EigenVector3 projectionContact(Contact *contact, const EigenVector3& momentum);
    EigenVector3 projectionFriction(Contact *contact, const EigenVector3& momentum, const EigenVector3 contact_impulse);
};
