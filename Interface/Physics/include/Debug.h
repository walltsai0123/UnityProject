#pragma once

#include "math_headers.h"
#include <iostream>
#include <fstream>

namespace Debug
{
    void PrintMatrixToLog(Eigen::MatrixXf& m)
    {
        std::cout << m << std::endl;
    }
    void PrintVectorToLog(Eigen::VectorXf& v)
    {   
        std::cout << v << std::endl;
    }
    template<typename Scalar>
    void PrintSTLVectorToLog(std::vector<Scalar> vec)
    {
        for(auto& v : vec)
            std::cout << v << std::endl;
    }
}
