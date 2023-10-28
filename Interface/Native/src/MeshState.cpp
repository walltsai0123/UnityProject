#include "MeshState.h"
#include "Util.h"
#include <fstream>

MeshState::MeshState(const MeshDataNative udata)
{
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

TetMeshState::TetMeshState(const TetMeshDataNative udata)
{
	VSize = udata.VSize;
	TSize = udata.TSize;

	V = new Eigen::MatrixXf(3, VSize);
	T = new Eigen::MatrixXi(4, TSize);

	// Copy over data
	MatrixFromMap(udata.VPtr, V);
	MatrixFromMap(udata.TPtr, T);

#ifndef NODEBUG
	using namespace std;
    ofstream logfile("log/TetMeshState.log");
	logfile << "MeshState\n";
    logfile << "V:\n";
    logfile << *V << "\n";
    logfile << "T:\n";
    logfile << *T << "\n";
    logfile.flush();
	logfile.close();
#endif
}

TetMeshState::~TetMeshState()
{
	delete V;
	delete T;
}
