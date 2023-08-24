#include "MeshState.h"
#include "Util.h"

MeshState::MeshState(const MeshDataNative udata)
{
	VSize = udata.VSize;
	FSize = udata.FSize;
	TSize = udata.TSize;

	V = new Eigen::MatrixXf(3, VSize);
	N = new Eigen::MatrixXf(3, VSize);
	C = new Eigen::MatrixXf(4, VSize);
	UV = new Eigen::MatrixXf(2, VSize);
	F = new Eigen::MatrixXi(3, FSize);
	T = new Eigen::MatrixXi(4, TSize);

	// Copy over data
	MatrixFromMap(udata.VPtr, V);
	MatrixFromMap(udata.NPtr, N);
	MatrixFromMap(udata.CPtr, C);
	MatrixFromMap(udata.UVPtr, UV);
	MatrixFromMap(udata.FPtr, F);
	MatrixFromMap(udata.TPtr, T);
}

MeshState::~MeshState()
{
	delete V;
	delete N;
	delete C;
	delete UV;
	delete F;
	delete T;
	// delete Native;
}