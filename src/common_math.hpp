/**
 * @file common_math.hpp
 * @author ponomarevda96@gmail.com
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
}  // namespace Math

#endif  // COMMON_MATH_HPP
