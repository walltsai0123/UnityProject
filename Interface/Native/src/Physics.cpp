#include "Backend.h"

#include "XPBD/XPBDRigidBody.h"
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

int AddXPBDRigidBody(Vector3 pos, Quaternion rot, Vector3 inertia, float mass)
{
    if(!xpbdSim)
	{
		xpbdSim.reset(new XPBDSimulation());
	}

	XPBDRigidBody *newRigid = new XPBDRigidBody(pos.AsEigen(), rot.AsEigen(), inertia.AsEigen(), mass);
	int ID = xpbdSim->AddBody(newRigid);
	return ID;
}

int AddXPBDRigidBox(Vector3 pos, Quaternion rot, Vector3 size, float mass)
{
    if(!xpbdSim)
	{
		xpbdSim.reset(new XPBDSimulation());
	}

	XPBDRigidBody *newRigid = new XPBDRigidBody(pos.AsEigen(), rot.AsEigen(), new Box(size.AsEigen()), mass);
	int ID = xpbdSim->AddBody(newRigid);
	return ID;
}

int AddXPBDRigidCylinder(Vector3 pos, Quaternion rot, float radius, float height, float mass)
{
	if(!xpbdSim)
	{
		xpbdSim.reset(new XPBDSimulation());
	}

	XPBDRigidBody *newRigid = new XPBDRigidBody(pos.AsEigen(), rot.AsEigen(), new Cylinder(radius, height), mass);
	int ID = xpbdSim->AddBody(newRigid);
	return ID;
}

int AddXPBDSoftBody(MeshState *meshState, TetMeshState *tetState, Vector3 pos, Quaternion rot, float mass, float mu, float lambda)
{
	if(!xpbdSim)
	{
		xpbdSim.reset(new XPBDSimulation());
	}

	XPBDSoftBody *newSoftBody = new XPBDSoftBody(pos.AsEigen(), rot.AsEigen(), meshState, tetState, mass, mu, lambda);
	int ID = xpbdSim->AddBody(newSoftBody);
	return ID;
}

void setBodyMaterial(int ID, float mu, float lambda)
{
	if(xpbdSim)
		xpbdSim->SetBodyMaterial(ID, mu, lambda);
}

void AddFixedJoint(int ID1, int ID2)
{
	if(xpbdSim)
		xpbdSim->AddFixedJoint(ID1, ID2);
}

void AttachRigidSoft(int rId, int sId)
{
	if(xpbdSim)
		xpbdSim->AttachRigidSoft(rId, sId);
}

void XPBDSimInit()
{
	xpbdSim.reset(new XPBDSimulation());
}

void XPBDSimUpdate(float dt, int substeps)
{
	if(xpbdSim)
		xpbdSim->Update(dt, substeps);
}

void XPBDSimDelete()
{
	xpbdSim.reset(nullptr);
}
