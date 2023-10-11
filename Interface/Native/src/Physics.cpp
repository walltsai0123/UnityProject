#include "Backend.h"

#include "XPBD/XPBDSoftBody.h"
#include "XPBD/XPBDSimulation.h"


std::unique_ptr<SoftBody> softbody(nullptr);
std::unique_ptr<XPBDSimulation> xpbdSim(nullptr);


void AddMesh(MeshState *meshState, const char *path,
            Vector3 pos, Quaternion rot,
            float mass, float mu, float lambda, int matType)
{
	softbody->AddMesh(meshState, path, pos.AsEigen(), rot.AsEigen(), mass, mu, lambda, matType);
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

void GetTransform(int index, Vector3 &position, Quaternion &rotation)
{
	EigenVector3 pos;
	Eigen::Quaternionf rot;
	//softbody->GetMeshTransform(index, pos, rot);
	xpbdSim->GetBodyTransform(index, pos, rot);

	position = Vector3(pos);
	rotation = Quaternion(rot);
}

void AddTorque(int index, float torque, Vector3 axis)
{
	softbody->AddTorque(index, torque, axis.AsEigen());
}

int AddXPBDSoftBody(MeshState *meshState, const char *path,
					Vector3 pos, Quaternion rot,
					float mass, float mu, float lambda)
{
	if(xpbdSim.get() == nullptr)
	{
		xpbdSim.reset(new XPBDSimulation());
	}

	XPBDSoftBody *newSoftBody = new XPBDSoftBody(meshState, path, pos.AsEigen(), rot.AsEigen(), mass, mu, lambda);
	int ID = xpbdSim->AddBody(newSoftBody);
	return ID;
}

void setBodyMaterial(int ID, float mu, float lambda)
{
	xpbdSim->SetBodyMaterial(ID, mu, lambda);
}

void AddPosConstraints(int ID1, int ID2, Vector3 R1, Vector3 R2, float len, float comp)
{
	xpbdSim->AddPosConstraint(ID1, ID2, R1.AsEigen(), R2.AsEigen(), len, comp);
}

void XPBDSimUpdate(float dt, int substeps)
{
	xpbdSim->Update(dt, substeps);
}

void XPBDSimDelete()
{
	xpbdSim.reset(nullptr);
}
