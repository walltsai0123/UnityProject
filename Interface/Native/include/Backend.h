#pragma once
#include <PluginAPI/IUnityInterface.h>
#include "BackendCallbacks.h"
#include "MeshState.h"
#include "PD/SoftBody.h"

extern std::unique_ptr<SoftBody> softbody;

extern "C"
{
    UNITY_INTERFACE_EXPORT void
    Initialize(StringCallback debugCallback, StringCallback debugWarningCallback, StringCallback debugErrorCallback);

    UNITY_INTERFACE_EXPORT
    MeshState *InitMeshState(const MeshDataNative data);

    UNITY_INTERFACE_EXPORT
    TetMeshState *InitTetMeshState(const TetMeshDataNative data);

    UNITY_INTERFACE_EXPORT
    void DisposeMeshState(MeshState *state);

    UNITY_INTERFACE_EXPORT
    void DisposeTetMeshState(TetMeshState *state);

    // Physics.cpp
    UNITY_INTERFACE_EXPORT
    void AddMesh(MeshState *meshState, const char *path,
                Vector3 pos, Quaternion rot,
                float mass, float mu, float lambda, int matType);

    UNITY_INTERFACE_EXPORT
    void AddContact(Vector3 p, Vector3 n, float seperation);

    UNITY_INTERFACE_EXPORT
    void CreateSoftBody();

    UNITY_INTERFACE_EXPORT
    void InitSoftBody();

    UNITY_INTERFACE_EXPORT
    void DeleteSoftBody();

    UNITY_INTERFACE_EXPORT
    void SimulationUpdate(float dt);

    UNITY_INTERFACE_EXPORT
    void GetTransform(int index, Vector3& position, Quaternion& rotation);

    UNITY_INTERFACE_EXPORT
    void AddTorque(int index, float torque, Vector3 axis);

    UNITY_INTERFACE_EXPORT
    int AddXPBDRigidBody(Vector3 pos, Quaternion rot, Vector3 inertia, float mass);

    UNITY_INTERFACE_EXPORT
    int AddXPBDSoftBody(MeshState *meshState, TetMeshState *tetState, Vector3 pos, Quaternion rot, float mass, float mu, float lambda);

    UNITY_INTERFACE_EXPORT
    void setBodyMaterial(int ID, float mu, float lambda);

    UNITY_INTERFACE_EXPORT
    void AddPosConstraints(int ID1, int ID2, Vector3 R1, Vector3 R2, float len, float comp);

    UNITY_INTERFACE_EXPORT
    void AddFixedJoint(int ID1, int ID2);

    UNITY_INTERFACE_EXPORT
    void XPBDSimUpdate(float dt, int substeps);

    UNITY_INTERFACE_EXPORT
    void XPBDSimDelete();


    // IO.cpp
    UNITY_INTERFACE_EXPORT
    void ApplyDirtyVis(MeshState *state, const MeshDataNative data);

    UNITY_INTERFACE_EXPORT
    void ApplyDirtyTet(TetMeshState *state, const TetMeshDataNative data);

    UNITY_INTERFACE_EXPORT
    bool ReadMESH(const char *path,
                  void *&VPtr, int &VSize,
                  void *&NPtr, int &NSize,
                  void *&FPtr, int &FSize,
                  void *&TPtr, int &TSize);

    UNITY_INTERFACE_EXPORT
    bool ReadPLY(const char *path,
                 void *&VPtr, int &VSize,
                 void *&NPtr, int &NSize,
                 void *&FPtr, int &FSize,
                 void *&UVPtr, int &UVSize);
}