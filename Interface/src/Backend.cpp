#include "Backend.h"
#include <igl/readOBJ.h>
#include <iostream>
#include <fstream>

SoftBody *softbody = nullptr;
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

	// Eigen::initParallel();
	// // remove Main, Render and Oculus thread
	// Eigen::setNbThreads(std::max(1, Eigen::nbThreads() - 3));
	softbody = new SoftBody();
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
	softbody->Init();
}
