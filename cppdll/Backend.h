#pragma once
#define NOMINMAX

#include <PluginAPI/IUnityInterface.h>
#include  "BackendCallbacks.h"
extern "C"
{
    UNITY_INTERFACE_EXPORT
        void Initialize(StringCallback debugCallback, StringCallback debugWarningCallback, StringCallback debugErrorCallback);

    // IO.cpp

    UNITY_INTERFACE_EXPORT
        bool ReadMESH(const char* path,
            void*& VPtr, int& VSize,
            void*& NPtr, int& NSize,
            void*& FPtr, int& FSize,
            void*& TPtr, int& TSize);

    UNITY_INTERFACE_EXPORT
        bool ReadPLY(const char* path,
            void*& VPtr, int& VSize,
            void*& NPtr, int& NSize,
            void*& FPtr, int& FSize,
            void*& UVPtr, int& UVSize);
}