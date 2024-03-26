#include "Backend.h"

#include <igl/readOBJ.h>
#include <iostream>
#include <fstream>
#include <omp.h>

// IO redirect
std::ofstream out("./log/cout.log");
std::ofstream err("./log/cerr.log");


void Initialize(const StringCallback debugCallback, StringCallback debugWarningCallback,
	StringCallback debugErrorCallback)
{
#ifndef NODEBUG
	DebugLog = debugCallback;
	DebugLogWarning = debugWarningCallback;
	DebugLogError = debugErrorCallback;

	// redirect standard output stream
	freopen("./log/stderr.log", "w", stderr);
	freopen("./log/stdout.log", "w", stdout);
	std::cout.rdbuf(out.rdbuf());
	std::cerr.rdbuf(err.rdbuf());
	fprintf(stderr, "Debug stdio redirect.\n");
	fprintf(stderr, "OpenMP max threads: %d\n", omp_get_max_threads());
#endif
	//Eigen::initParallel();
	LOG("Initialized BackEnd.")
}

