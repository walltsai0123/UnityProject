#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#define USE_DOUBLE

#ifdef USE_DOUBLE
typedef double REAL;
#define EPSILON 1e-10
#define EPSILON_SQUARE 1e-20
#else
typedef float REAL;
#define EPSILON 1e-6
#define EPSILON_SQUARE 1e-12
#endif

typedef vector <REAL, 2> REAL2;
typedef vector <REAL, 3> REAL3;
typedef vector <REAL, 4> REAL4;
typedef matrix <REAL, 2, 2> REAL2x2;
typedef matrix <REAL, 3, 3> REAL3x3;
typedef matrix <REAL, 3, 4> REAL3x4;
typedef matrix <REAL, 4, 4> REAL4x4;

#endif