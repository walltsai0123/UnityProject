#include "Backend.h"

std::unique_ptr<SoftBody> softbody(nullptr);


void AddMesh(MeshState *meshState, const char *path)
{
	softbody->AddMesh(meshState, path);
}

void AddContact(Vector3 p, Vector3 n, float seperation)
{
    softbody->AddContact(p.AsEigen(), n.AsEigen(), seperation);
}

void CreateSoftBody()
{
	softbody.reset(new SoftBody());
}

void InitSoftBody()
{
	softbody->Init();
}

void DeleteSoftBody()
{
	softbody.reset(nullptr);
}

void SimulationUpdate(float dt)
{
	softbody->Update(dt);
}

void CollisionUpdate()
{
	LOG("CollisionUpdate.");
}

void MeshesUpdate()
{
	softbody->UpdateMeshes();
}
