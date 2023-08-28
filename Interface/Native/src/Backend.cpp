#include "Backend.h"

#include <igl/readOBJ.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <omp.h>

std::unique_ptr<SoftBody> softbody(nullptr);

// IO redirect
std::ofstream out("./log/cout.log");
std::ofstream err("./log/cerr.log");

void Initialize(const StringCallback debugCallback, StringCallback debugWarningCallback,
                StringCallback debugErrorCallback)
{
#ifndef NDEBUG
	DebugLog = debugCallback;
	DebugLogWarning = debugWarningCallback;
	DebugLogError = debugErrorCallback;

	// redirect standard output stream
	freopen("./log/stderr.log", "w", stderr);
	freopen("./log/stdout.log", "w", stdout);
	std::cout.rdbuf(out.rdbuf());
	std::cerr.rdbuf(err.rdbuf());
	fprintf(stderr, "Debug stdio redirect.\n");
#endif
	// int max_threads = omp_get_max_threads();
	// omp_set_num_threads(std::max(1, max_threads - 2));
	// fprintf(stderr, "OpenMP Max threads: %d\n", omp_get_max_threads());
	LOG("Initialized BackEnd.")
}

MeshState *InitMeshState(const MeshDataNative data)
{
	auto *state = new MeshState(data);
    return state;
}

void DisposeMeshState(MeshState *state)
{
	delete state;
}

void AddMesh(MeshState *meshState)
{
	softbody->AddMesh(meshState);
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

void SimulationUpdate()
{
	softbody->Update();
}

void MeshesUpdate()
{
	softbody->UpdateMeshes();
}
