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


#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP

#include <Eigen/Geometry>

namespace Math
{
    /**
    * @note https://en.wikipedia.org/wiki/Linear_interpolation
    */
    double lerp(double a, double b, double f);

    double polyval(const Eigen::VectorXd& poly, double val);

    /**
     * @brief Given monotonic sequence (increasing or decreasing) and key,
     return the index of the previous element closest to the key
     * @note size should be greater or equel than 2!
     */
    size_t findPrevRowIdxInMonotonicSequence(const Eigen::MatrixXd& matrix, double key);

    /**
     * @brief Given an increasing sequence and a key,
     return the index of the previous element closest to the key
     * @note size should be greater or equel than 2!
     */
    size_t findPrevRowIdxInIncreasingSequence(const Eigen::MatrixXd& table, double value);

    /**
     * @note Similar to https://www.mathworks.com/help/matlab/ref/griddata.html
     * Implementation from https://en.wikipedia.org/wiki/Bilinear_interpolation
     */
    double griddata(const Eigen::MatrixXd& x,
                    const Eigen::MatrixXd& y,
                    const Eigen::MatrixXd& z,
                    double x_val,
                    double y_val);

    /**
     * @param[in] table must have size (1 + NUM_OF_COEFFS, NUM_OF_POINTS), min size is (2, 2)
     * @param[in] airSpeedMod should be between table(0, 0) and table(NUM_OF_COEFFS, 0)
     * @param[in, out] polynomialCoeffs must have size should be at least NUM_OF_COEFFS
     * @return true and modify polynomialCoeffs if input is ok, otherwise return false
     */
    bool calculatePolynomial(const Eigen::MatrixXd& table,
                             double airSpeedMod,
                             Eigen::VectorXd& polynomialCoeffs);


}  // namespace Math

#endif  // COMMON_MATH_HPP
