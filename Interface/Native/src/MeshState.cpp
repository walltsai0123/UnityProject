#include "MeshState.h"
#include "Util.h"
#include <fstream>

MeshState::MeshState(const MeshDataNative udata)
{
	Mass = udata.Mass;
	com = udata.com;
	
	materialType = udata.materialType;
	mu = udata.mu;
	lambda = udata.lambda;
	
	VSize = udata.VSize;
	FSize = udata.FSize;

	V = new Eigen::MatrixXf(3, VSize);
	N = new Eigen::MatrixXf(3, VSize);
	F = new Eigen::MatrixXi(3, FSize);

	// Copy over data
	MatrixFromMap(udata.VPtr, V);
	MatrixFromMap(udata.NPtr, N);
	MatrixFromMap(udata.FPtr, F);

#ifndef NODEBUG
	using namespace std;
    ofstream logfile("log/MeshState.log");
	logfile << "MeshState\n";
    logfile << "V:\n";
    logfile << *V << "\n";
    logfile << "N:\n";
    logfile << *N << "\n";
	logfile << "F\n";
    logfile << *F << "\n";
	logfile << "materialType: " << materialType << "\n";
    logfile << "mu, lambda: " << mu << " " << lambda << "\n";
    logfile.flush();
	logfile.close();
#endif
}

MeshState::~MeshState()
{
	delete V;
	delete N;
	delete F;
}