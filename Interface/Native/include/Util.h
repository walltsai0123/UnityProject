#pragma once
#include <iostream>

template<typename Scalar, typename Matrix>
void MatrixFromMap(Scalar* from, Matrix* to)
{
	auto fromMap = Eigen::Map<Matrix>(from, to->rows(), to->cols());
	*to = fromMap;
}

template<typename Scalar, typename Matrix>
void MatrixToMap(Matrix* from, Scalar* to)
{
	auto toMap = Eigen::Map<Matrix>(to, from->rows(), from->cols());
	toMap = *from;
}