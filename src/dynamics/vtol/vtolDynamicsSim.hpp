/**
 * @file vtolDynamicsSim.hpp
 * @author ponomarevda96@gmail.com
 * @brief Vtol dynamics simulator class header file
 */

#ifndef VTOL_DYNAMICS_SIM_H
#define VTOL_DYNAMICS_SIM_H

#include <Eigen/Geometry>
#include <vector>
#include <array>
#include <random>
#include "uavDynamicsSimBase.hpp"


struct VtolParameters{
    double mass;                                    // kg
    double gravity;                                 // n/sec^2
    double atmoRho;                                 // air density (kg/m^3)
    double wingArea;                                // m^2
    double characteristicLength;                    // m
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> inertia;   // kg*m^2

    std::array<Eigen::Vector3d, 5> propellersLocation;

    std::vector<double> actuatorMin;                // rad/sec
    std::vector<double> actuatorMax;                // rad/sec
    std::array<double, 8> deltaControlMax;          // rad/sec^2
    std::array<double, 8> timeConstant;             // sec

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
    std::array<Eigen::Vector3d, 5> motors;
    Eigen::Vector3d specific;
    Eigen::Vector3d total;
};

struct Moments{
    Eigen::Vector3d aero;
    Eigen::Vector3d steer;
    Eigen::Vector3d airspeed;
    std::array<Eigen::Vector3d, 5> motors;
    Eigen::Vector3d total;
};

struct State{
    /**
     * @note Inertial frame (NED)
     */
    Eigen::Vector3d initialPose;                    // meters
    Eigen::Vector3d position;                       // meters
    Eigen::Vector3d linearVel;                      // m/sec
    Eigen::Vector3d linearAccel;                    // m/sec^2

    /**
     * @note Body frame (FRD)
     */
    Eigen::Quaterniond initialAttitude;             // quaternion
    Eigen::Quaterniond attitude;                    // quaternion
    Eigen::Vector3d angularVel;                     // rad/sec
    Eigen::Vector3d angularAccel;                   // rad/sec^2

    Forces forces;
    Moments moments;

    std::array<double, 5> motorsRpm;                // rpm
    Eigen::Vector3d bodylinearVel;                  // m/sec (just for debug only)
    std::vector<double> prevActuators;              // rad/sec
    std::vector<double> crntActuators;              // rad/sec
};

struct Environment{
    double windVariance;
    Eigen::Vector3d windVelocity;                   // m/sec^2
    Eigen::Vector3d gustVelocity;                   // m/sec^2
    double gustVariance;
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
class InnoVtolDynamicsSim : public UavDynamicsSimBase{
    public:
        InnoVtolDynamicsSim();
        ~InnoVtolDynamicsSim() final = default;

        int8_t init() override;
        void setInitialPosition(const Eigen::Vector3d & position,
                                const Eigen::Quaterniond& attitude) override;
        void land() override;
        int8_t calibrate(SimMode_t calibrationType) override;
        void process(double dt_secs,
                     const std::vector<double>& motorSpeedCommandIn,
                     bool isCmdPercent) override;

        /**
         * @note These methods should return in NED format
         */
        Eigen::Vector3d getVehiclePosition() const override;
        Eigen::Quaterniond getVehicleAttitude() const override;
        Eigen::Vector3d getVehicleVelocity() const override;
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
                               const std::vector<double>& actuator,
                               double dt_sec);

        void calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                   double AoA,
                                   double AoS,
                                   double aileron_pos,
                                   double elevator_pos,
                                   double rudder_pos,
                                   Eigen::Vector3d& Faero,
                                   Eigen::Vector3d& Maero);

        void calculateCLPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCSPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCDPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmxPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmyPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;
        void calculateCmzPolynomial(double airSpeedMod, Eigen::VectorXd& polynomialCoeffs) const;

        /**
         * @param[in] table must have size (1 + NUM_OF_COEFFS, NUM_OF_POINTS), min size is (2, 2)
         * @param[in] airSpeedMod should be between table(0, 0) and table(NUM_OF_COEFFS, 0)
         * @param[in, out] polynomialCoeffs must have size should be at least NUM_OF_COEFFS
         * @return true and modify polynomialCoeffs if input is ok, otherwise return false
         */
        bool calculatePolynomialUsingTable(const Eigen::MatrixXd& table,
                                           double airSpeedMod,
                                           Eigen::VectorXd& polynomialCoeffs) const;

        double calculateCSRudder(double rudder_pos, double airspeed) const;
        double calculateCSBeta(double AoS_deg, double airspeed) const;
        double calculateCmxAileron(double aileron_pos, double airspeed) const;
        double calculateCmyElevator(double elevator_pos, double airspeed) const;
        double calculateCmzRudder(double rudder_pos, double airspeed) const;

        Eigen::Vector3d calculateAngularAccel(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& inertia,
                                              const Eigen::Vector3d& moment,
                                              const Eigen::Vector3d& prevAngVel) const;

        void setWindParameter(Eigen::Vector3d windMeanVelocity, double wind_velocityVariance);
        void setInitialVelocity(const Eigen::Vector3d& linearVelocity,
                                const Eigen::Vector3d& angularVelocity);

    private:
        void loadTables(const std::string& path);
        void loadParams(const std::string& path);
        std::vector<double> mapCmdToActuatorStandardVTOL(const std::vector<double>& cmd) const;
        std::vector<double> mapCmdToActuatorInnoVTOL(const std::vector<double>& cmd) const;
        void updateActuators(std::vector<double>& cmd, double dtSecs);
        Eigen::Vector3d calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                    const Eigen::Vector3d& estimatedVelocity,
                                    const Eigen::Vector3d& windSpeed) const;

        VtolParameters params_;
        State state_;
        TablesWithCoeffs tables_;
        Environment environment_;

        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_{0.0, 1.0};
};

#endif  // VTOL_DYNAMICS_SIM_H
