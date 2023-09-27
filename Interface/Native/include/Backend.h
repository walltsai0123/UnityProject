#pragma once
#include <PluginAPI/IUnityInterface.h>
#include "BackendCallbacks.h"
#include "MeshState.h"
#include "SoftBody.h"

extern std::unique_ptr<SoftBody> softbody;

extern "C"
{
    UNITY_INTERFACE_EXPORT void
    Initialize(StringCallback debugCallback, StringCallback debugWarningCallback, StringCallback debugErrorCallback);

    UNITY_INTERFACE_EXPORT
    MeshState *InitMeshState(const MeshDataNative data);

    UNITY_INTERFACE_EXPORT
    void DisposeMeshState(MeshState *state);

    // Physics.cpp
    UNITY_INTERFACE_EXPORT
    void AddMesh(MeshState *meshState, const char *path);

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
    void CollisionUpdate();

    UNITY_INTERFACE_EXPORT
    void MeshesUpdate();

    // IO.cpp
    UNITY_INTERFACE_EXPORT
    void ApplyDirty(MeshState *state, const MeshDataNative data);

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