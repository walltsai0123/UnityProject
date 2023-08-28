#pragma once

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>


#ifdef HIGH_PRECISION
	typedef double ScalarType;
	typedef double REAL;
	#define TW_TYPE_SCALAR_TYPE TW_TYPE_DOUBLE
#else
	typedef float ScalarType;
	#define TW_TYPE_SCALAR_TYPE TW_TYPE_FLOAT
#endif

// small number and large number
#ifdef HIGH_PRECISION
	#define EPSILON 1e-15
	#define EPSILON_SQUARE 1e-30
	#define LARGER_EPSILON 1e-6
#else
	#define EPSILON 1e-6
	#define EPSILON_SQUARE 1e-12
	#define LARGER_EPSILON 1e-4
#endif

// eigen vectors and matrices
typedef int IndexType;
typedef Eigen::Matrix<ScalarType, 9,  12, 0, 9, 12> EigenMatrix9x12;
typedef Eigen::Matrix<ScalarType, 12, 12, 0, 12, 12> EigenMatrix12;
typedef Eigen::Matrix<ScalarType, 12, 1, 0, 12, 1> EigenVector12;
typedef Eigen::Matrix<ScalarType, 9, 1, 0, 9, 1> EigenVector9;
typedef Eigen::Matrix<ScalarType, 4, 4, 0, 4, 4> EigenMatrix4;
typedef Eigen::Matrix<ScalarType, 4, 1, 0, 4, 1> EigenVector4;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3, 3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3 ,1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2 ,2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2 ,1> EigenVector2;
typedef Eigen::Matrix<ScalarType, -1, 3, 0, -1 ,3> EigenMatrixx3;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXX;
typedef Eigen::SparseMatrix<ScalarType> SparseMatrix;
typedef Eigen::Triplet<ScalarType,IndexType> SparseMatrixTriplet;

// int eigen matrix/vector
typedef Eigen::Matrix<IndexType, 2, 1> EigenVector2I;
typedef Eigen::Matrix<IndexType, 3, 1> EigenVector3I;
typedef Eigen::Matrix<IndexType, 4, 1> EigenVector4I;
typedef Eigen::Matrix<IndexType, Eigen::Dynamic, Eigen::Dynamic> MatrixI;

// eigen quaternions
typedef Eigen::AngleAxis<ScalarType> EigenAngleAxis;
typedef Eigen::Quaternion<ScalarType, Eigen::DontAlign> EigenQuaternion;

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)

// convert between Vector (3m * 1) and Matrix (m * 3)
void Vector3mx1ToMatrixmx3(const VectorX&x, EigenMatrixx3& m);
void Matrixmx3ToVector3mx1(const EigenMatrixx3& m, VectorX&x);
// convert a MATRIX3 to a VECTOR9 in a consistent way
EigenVector9 flatten(const EigenMatrix3& A);

// eigen make sparse identity
void EigenMakeSparseIdentityMatrix(unsigned int rows, unsigned int cols, SparseMatrix& I);

template<typename T, int Options>
Eigen::Quaternion<T, Options> operator*(T a, const Eigen::Quaternion<T, Options>& q)
{
    return Eigen::Quaternion<T, Options>(a*q.w(),a*q.x(),a*q.y(),a*q.z());
}

template<typename T, int Options>
Eigen::Quaternion<T, Options> operator+(const Eigen::Quaternion<T, Options>& q1, const Eigen::Quaternion<T, Options>& q2)
{
    return Eigen::Quaternion<T, Options>(q1.w()+q2.w(),q1.x()+q2.x(),q1.y()+q2.y(),q1.z()+q2.z());
}


// class QueueLBFGS
// {
// public:
// 	QueueLBFGS(unsigned int vector_size, unsigned int queue_size);
// 	~QueueLBFGS();

// 	inline int capacity() { return m_capacity; }
// 	inline bool isFull() { return m_is_full; }

// 	int size();
// 	void enqueue(const VectorX& sk, const VectorX& yk);
// 	void dequeue();

// 	void visitSandY(ScalarType** s, ScalarType** y, int i);

// protected:
// 	inline void checkEmpty() { m_is_empty = (m_head_pointer == m_tail_pointer) ? true : false; }
// 	inline void checkFull() { m_is_full = (m_head_pointer == m_tail_pointer) ? true : false; }

// protected:
// 	ScalarType* m_data_s;
// 	ScalarType* m_data_y;

// 	bool m_is_empty;
// 	bool m_is_full;

// 	int m_vector_size;
// 	int m_capacity;
// 	int m_head_pointer;
// 	int m_tail_pointer;
// };
