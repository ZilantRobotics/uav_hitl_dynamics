/*
 * Copyright (c) 2020-2023 RaccoonLab.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
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

double griddata(const Eigen::MatrixXd& x,
                const Eigen::MatrixXd& y,
                const Eigen::MatrixXd& z,
                double x_val,
                double y_val){
    size_t x1_idx = findPrevRowIdxInMonotonicSequence(x, x_val);
    size_t y1_idx = findPrevRowIdxInMonotonicSequence(y, y_val);
    size_t x2_idx = x1_idx + 1;
    size_t y2_idx = y1_idx + 1;
    double Q11 = z(y1_idx, x1_idx);
    double Q12 = z(y2_idx, x1_idx);
    double Q21 = z(y1_idx, x2_idx);
    double Q22 = z(y2_idx, x2_idx);
    double R1 = ((x(x2_idx) - x_val) * Q11 + (x_val - x(x1_idx)) * Q21) / (x(x2_idx) - x(x1_idx));
    double R2 = ((x(x2_idx) - x_val) * Q12 + (x_val - x(x1_idx)) * Q22) / (x(x2_idx) - x(x1_idx));
    double f =  ((y(y2_idx) - y_val) * R1  + (y_val - y(y1_idx)) * R2)  / (y(y2_idx) - y(y1_idx));
    return f;
}

bool calculatePolynomial(const Eigen::MatrixXd& table,
                         double airSpeedMod,
                         Eigen::VectorXd& polynomialCoeffs){
    if(table.cols() < 2 || table.rows() < 2 || polynomialCoeffs.rows() < table.cols() - 1){
        return false;  // wrong input
    }

    const size_t prevRowIdx = Math::findPrevRowIdxInMonotonicSequence(table, airSpeedMod);
    if(prevRowIdx + 2 > table.rows()){
        return false;  // wrong found row
    }

    const size_t nextRowIdx = prevRowIdx + 1;
    const double airspeedStep = table.row(nextRowIdx)(0, 0) - table.row(prevRowIdx)(0, 0);
    if (abs(airspeedStep) < 0.001) {
        return false;  // wrong table, prevent division on zero
    }

    double delta = (airSpeedMod - table.row(prevRowIdx)(0, 0)) / airspeedStep;
    const size_t numberOfCoeffs = table.cols() - 1;
    for(size_t coeff_idx = 0; coeff_idx < numberOfCoeffs; coeff_idx++){
        const double prevValue = table.row(prevRowIdx)(0, coeff_idx + 1);
        const double nextValue = table.row(nextRowIdx)(0, coeff_idx + 1);
        polynomialCoeffs[coeff_idx] = Math::lerp(prevValue, nextValue, delta);
    }

    return true;
}

}  // namespace Math
