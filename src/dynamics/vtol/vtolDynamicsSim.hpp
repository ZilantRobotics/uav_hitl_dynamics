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


#ifndef VTOL_DYNAMICS_SIM_H
#define VTOL_DYNAMICS_SIM_H

#include <Eigen/Geometry>
#include <vector>
#include <array>
#include <random>
#include "uavDynamicsSimBase.hpp"

inline constexpr size_t MOTORS_MIN_AMOUNT = 5;
inline constexpr size_t MOTORS_MAX_AMOUNT = 9;

struct Geometry {
    Eigen::Vector3d position;                       // Meters
    Eigen::Vector3d axis;                           // Unitless
    bool directionCCW;                              // True for CCW, False for CW
};

struct VtolParameters{
    double mass;                                    // kg
    double wingArea;                                // m^2
    double characteristicLength;                    // m
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> inertia;   // kg*m^2

    std::vector<double> actuatorMin;                // rad/sec
    std::vector<double> actuatorMax;                // rad/sec

    std::vector<double> motorMaxSpeed;              // rad/sec
    std::vector<double> servoRange;
    std::vector<Geometry> geometry;

    double accVariance;
    double gyroVariance;

    /**
     * @note not ready yet
     */
    double massUncertainty;                         // multiplier
    double inertiaUncertainty;                      // multiplier

    Eigen::Vector3d accelBias;
    Eigen::Vector3d gyroBias;
};

struct Forces{
    Eigen::Vector3d lift;
    Eigen::Vector3d drug;
    Eigen::Vector3d side;
    Eigen::Vector3d aero;
    std::array<Eigen::Vector3d, MOTORS_MAX_AMOUNT> motors;
    Eigen::Vector3d specific;
    Eigen::Vector3d total;
};

struct Moments{
    Eigen::Vector3d aero;
    Eigen::Vector3d steer;
    Eigen::Vector3d airspeed;
    std::array<Eigen::Vector3d, MOTORS_MAX_AMOUNT> motors;
    Eigen::Vector3d total;
};

struct State{
    /**
     * @note Inertial frame (NED)
     */
    Eigen::Vector3d initialPose;                    // meters
    Eigen::Vector3d position;                       // meters
    Eigen::Vector3d linearVelNed;                   // m/sec
    Eigen::Vector3d linearAccel;                    // m/sec^2

    /**
     * @note Body frame (FRD)
     */
    Eigen::Quaterniond initialAttitude;             // quaternion
    Eigen::Quaterniond attitude;                    // quaternion
    Eigen::Vector3d angularVel;                     // rad/sec
    Eigen::Vector3d angularAccel;                   // rad/sec^2
    Eigen::Vector3d airspeedFrd;                    // m/sec

    Forces forces;
    Moments moments;

    std::array<double, MOTORS_MAX_AMOUNT> motorsRpm;  // rpm
    Eigen::Vector3d bodylinearVel;                  // m/sec (just for debug only)
    std::vector<double> prevActuators;              // rad/sec
    std::vector<double> crntActuators;              // rad/sec
};

struct Environment{
    double windVariance;
    Eigen::Vector3d windNED;                        // m/sec^2
    Eigen::Vector3d gustVelocityNED;                // m/sec^2
    double gustVariance;
    double gravity;                                 // n/sec^2
    double atmoRho;                                 // air density (kg/m^3)
};

struct TablesWithCoeffs{
    Eigen::Matrix<double, 8, 20, Eigen::RowMajor> CS_rudder;
    Eigen::Matrix<double, 8, 90, Eigen::RowMajor> CS_beta;

    Eigen::Matrix<double, 1, 47, Eigen::RowMajor> AoA;
    Eigen::Matrix<double, 90, 1, Eigen::ColMajor> AoS;

    Eigen::Matrix<double, 20, 1, Eigen::ColMajor> actuator;
    Eigen::Matrix<double, 8, 1, Eigen::ColMajor> airspeed;

    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> CLPolynomial;
    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> CSPolynomial;
    Eigen::Matrix<double, 8, 6, Eigen::RowMajor> CDPolynomial;
    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> CmxPolynomial;
    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> CmyPolynomial;
    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> CmzPolynomial;

    Eigen::Matrix<double, 8, 20, Eigen::RowMajor> CmxAileron;
    Eigen::Matrix<double, 8, 20, Eigen::RowMajor> CmyElevator;
    Eigen::Matrix<double, 8, 20, Eigen::RowMajor> CmzRudder;

    Eigen::Matrix<double, 40, 5, Eigen::RowMajor> prop;

    std::vector<double> actuatorTimeConstants;
};

/**
 * @brief Vtol dynamics simulator class
 */
class VtolDynamics : public UavDynamicsSimBase{
    public:
        VtolDynamics();
        ~VtolDynamics() final = default;

        int8_t init() override;
        void setInitialPosition(const Eigen::Vector3d & position,
                                const Eigen::Quaterniond& attitudeXYZW) override;
        void land() override;
        int8_t calibrate(SimMode_t calibrationType) override;
        void process(double dt_secs, const std::vector<double>& unitless_setpoint) override;

        /**
         * @note These methods should return in NED format
         */
        Eigen::Vector3d getVehiclePosition() const override;
        Eigen::Quaterniond getVehicleAttitude() const override;
        Eigen::Vector3d getVehicleVelocity() const override;
        Eigen::Vector3d getVehicleAirspeed() const override;
        Eigen::Vector3d getVehicleAngularVelocity() const override;
        void getIMUMeasurement(Eigen::Vector3d& accOut, Eigen::Vector3d& gyroOut) override;
        bool getMotorsRpm(std::vector<double>& motorsRpm) override;

        /**
         * @note For RVIZ visualization only
         */
        const Forces& getForces() const;
        const Moments& getMoments() const;
        Eigen::Vector3d getBodyLinearVelocity() const;

        // For tests only
        Eigen::Vector3d getAngularAcceleration() const;
        Eigen::Vector3d getLinearAcceleration() const;

        /**
         * @note The methods below are should be public for test only
         * think about making test as friend
         */
        Eigen::Vector3d calculateNormalForceWithoutMass() const;
        Eigen::Vector3d calculateWind();
        Eigen::Matrix3d calculateRotationMatrix() const;
        double calculateDynamicPressure(double airSpeedMod) const;
        double calculateAnglesOfAtack(const Eigen::Vector3d& airSpeed) const;
        double calculateAnglesOfSideslip(const Eigen::Vector3d& airSpeed) const;
        void thruster(double actuator, double& thrust, double& torque, double& rpm) const;
        void calculateNewState(const Eigen::Vector3d& Maero,
                               const Eigen::Vector3d& Faero,
                               const std::vector<double>& motors,
                               double dt_sec);

        void calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                   double AoA,
                                   double AoS,
                                   const std::array<double, 3>& servos,
                                   Eigen::Vector3d& Faero,
                                   Eigen::Vector3d& Maero);

        void calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;

        double calculateCSRudder(double rudder_pos, double airspeed) const;
        double calculateCSBeta(double AoS_deg, double airspeed) const;
        double calculateCmxAileron(double aileron_pos, double airspeed) const;
        double calculateCmyElevator(double elevator_pos, double airspeed) const;
        double calculateCmzRudder(double rudder_pos, double airspeed) const;

        Eigen::Vector3d calculateAngularAccel(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& inertia,
                                              const Eigen::Vector3d& moment,
                                              const Eigen::Vector3d& prevAngVel) const;

        void setWindParameter(Eigen::Vector3d windMeanVelocityNED, double wind_velocityVariance) override;
        void setInitialVelocity(const Eigen::Vector3d& linearVelocity,
                                const Eigen::Vector3d& angularVelocity);

    private:
        void loadTables(const std::string& path);
        void loadParams(const std::string& path);
        void loadMotorsGeometry(const std::string& path);
        void _mapUnitlessSetpointToInternal(const std::vector<double>& cmd);
        void updateActuators(double dtSecs);
        Eigen::Vector3d calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                          const Eigen::Vector3d& estimatedVelocity,
                                          const Eigen::Vector3d& windSpeed) const;

        std::vector<double> _motorsSpeed;
        std::array<double, 3> _servosValues{0.0, 0.0, 0.0};

        VtolParameters _params;
        State _state;
        TablesWithCoeffs _tables;
        Environment _environment;

        std::default_random_engine _generator;
        std::normal_distribution<double> _distribution{0.0, 1.0};
};

#endif  // VTOL_DYNAMICS_SIM_H
