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
     * @todo think about binary search
     */
    size_t findPrevRowIdxInMonotonicSequence(const Eigen::MatrixXd& matrix, double key);

    /**
     * @brief Given an increasing sequence and a key,
     return the index of the previous element closest to the key
     * @note size should be greater or equel than 2!
     * @todo think about binary search
     */
    size_t findPrevRowIdxInIncreasingSequence(const Eigen::MatrixXd& table, double value);
}

#endif  // COMMON_MATH_HPP
