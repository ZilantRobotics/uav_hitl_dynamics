/**
 * @file common_math.cpp
 * @author ponomarevda96@gmail.com
 */

#include "common_math.hpp"
#include <Eigen/Geometry>

namespace Math
{

double lerp(double a, double b, double f){
    return a + f * (b - a);
}

double polyval(const Eigen::VectorXd& poly, double val){
    double result = 0;
    for(uint8_t idx = 0; idx < poly.rows(); idx++){
        result += poly[idx] * std::pow(val, poly.rows() - 1 - idx);
    }
    return result;
}

size_t findPrevRowIdxInMonotonicSequence(const Eigen::MatrixXd& matrix, double key){
    size_t row_idx;
    bool is_increasing_sequence = matrix(matrix.rows() - 1, 0) > matrix(0, 0);
    if(is_increasing_sequence){
        for(row_idx = 1; row_idx < matrix.rows() - 1; row_idx++){
            if(key <= matrix(row_idx, 0)){
                break;
            }
        }
        row_idx--;
    }else{
        for(row_idx = 1; row_idx < matrix.rows() - 1; row_idx++){
            if(key >= matrix(row_idx, 0)){
                break;
            }
        }
        row_idx--;
    }
    return row_idx;
}

size_t findPrevRowIdxInIncreasingSequence(const Eigen::MatrixXd& table, double value){
    size_t row_idx = 0;
    size_t num_of_rows = table.rows();
    while(row_idx + 2 < num_of_rows && table(row_idx + 1, 0) < value){
        row_idx++;
    }
    return row_idx;
}

}
