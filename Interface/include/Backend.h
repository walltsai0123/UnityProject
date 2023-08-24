#pragma once
#include <PluginAPI/IUnityInterface.h>
#include "BackendCallbacks.h"
#include "MeshState.h"
#include "Physics/SoftBody.h"

extern "C"
{
    UNITY_INTERFACE_EXPORT void
    Initialize(StringCallback debugCallback, StringCallback debugWarningCallback, StringCallback debugErrorCallback);

    UNITY_INTERFACE_EXPORT
    MeshState *InitMeshState(const MeshDataNative data);

    UNITY_INTERFACE_EXPORT
    void DisposeMeshState(MeshState *state);

    UNITY_INTERFACE_EXPORT
    void AddMesh(MeshState* meshState);

    // IO.cpp
    UNITY_INTERFACE_EXPORT
    bool ReadMESH(const char *path,
                void *&VPtr, int &VSize,
                void *&NPtr, int &NSize,
                void *&FPtr, int &FSize,
                void *&TPtr, int &TSize);

}