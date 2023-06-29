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


#ifndef MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP
#define MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP

#include "uavDynamicsSimBase.hpp"
#include "../libs/multicopterDynamicsSim/multicopterDynamicsSim.hpp"

class FlightgogglesDynamics: public UavDynamicsSimBase{
public:
    FlightgogglesDynamics() = default;
    ~FlightgogglesDynamics() final = default;

    int8_t init() override;
    void setInitialPosition(const Eigen::Vector3d & position,
                            const Eigen::Quaterniond& attitude) override;

    void process(double dt_secs, const std::vector<double> & motorSpeedCommandIn, bool isCmdPercent) override;

    Eigen::Vector3d getVehiclePosition() const override;
    Eigen::Quaterniond getVehicleAttitude() const override;
    Eigen::Vector3d getVehicleVelocity(void) const override;
    Eigen::Vector3d getVehicleAngularVelocity(void) const override;
    void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) override;

private:
    std::unique_ptr<MulticopterDynamicsSim> multicopterSim_;

    void initStaticMotorTransform();

    /**
     * @brief Convert actuator indexes from PX4 notation to internal Flightgoggles notation
     * @param cmd with indexes: 0 - front right, 1 - tail left, 2 - front left, 3 - tail right
     * @return cmd with indexes: 0 - front left, 1 - tail left, 2 - tail right, 3 - front right
     */
    std::vector<double> mapCmdActuator(std::vector<double> cmd) const;
};

#endif  // MULTICOPTER_DYNAMICS_WRAPPER_BASE_HPP
