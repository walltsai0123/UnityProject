#pragma once
#include <iostream>

template<typename Scalar, typename Matrix>
void MatrixFromMap(Scalar* from, Matrix* to)
{
	auto fromMap = Eigen::Map<Matrix>(from, to->rows(), to->cols());
	*to = fromMap;

}

template<typename Scalar, typename Matrix>
void TransposeFromMap(Scalar* from, Matrix* to)
{
	auto fromMap = Eigen::Map<Matrix>(from, to->cols(), to->rows());
	*to = fromMap.transpose();

}