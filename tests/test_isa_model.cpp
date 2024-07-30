/*
 * Copyright (c) 2022-2023 RaccoonLab.
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

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "sensors_isa_model.hpp"
#include <ros/ros.h>


TEST(IsaModel, diffPressureHpaMax){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0);
    Eigen::Vector3d linVelNed(100, 0, 0);
    double expectedDiffPressureHpa = 61.25;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}
TEST(IsaModel, diffPressureHpaZero){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0);
    Eigen::Vector3d linVelNed(0, 0, 0);

    double expectedDiffPressureHpa = 0;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}
TEST(IsaModel, diffPressureHpaMin){
    Eigen::Vector3d gpsPosition(55.7544426, 48.742684, 0);
    Eigen::Vector3d linVelNed(-100, 0, 0);

    double expectedDiffPressureHpa = 61.25;

    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                        temperatureKelvin, absPressureHpa, diffPressureHpa);
    EXPECT_NEAR(diffPressureHpa, expectedDiffPressureHpa, 0.01);
}


int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "isa_model_test");
    return RUN_ALL_TESTS();
}
