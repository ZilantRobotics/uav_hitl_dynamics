/* 
 * Copyright (c) 2020-2022 RaccoonLab.
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

#ifndef SENSORS_ISA_MODEL_HPP
#define SENSORS_ISA_MODEL_HPP

#include <Eigen/Geometry>

namespace SensorModelISA
{

    void EstimateAtmosphere(const Eigen::Vector3d& gpsPosition, const Eigen::Vector3d& linVelNed,
                        float& temperatureKelvin, float& absPressureHpa, float& diffPressureHpa){
        const float PRESSURE_MSL_HPA = 1013.250f;
        const float TEMPERATURE_MSL_KELVIN = 288.0f;
        const float RHO_MSL = 1.225f;
        const float LAPSE_TEMPERATURE_RATE = 1 / 152.4;

        float alt_msl = gpsPosition.z();

        temperatureKelvin = TEMPERATURE_MSL_KELVIN - LAPSE_TEMPERATURE_RATE * alt_msl;
        float pressureRatio = powf((TEMPERATURE_MSL_KELVIN/temperatureKelvin), 5.256f);
        const float densityRatio = powf((TEMPERATURE_MSL_KELVIN/temperatureKelvin), 4.256f);
        float rho = RHO_MSL / densityRatio;
        absPressureHpa = PRESSURE_MSL_HPA / pressureRatio;
        diffPressureHpa = 0.005f * rho * (float)(linVelNed.norm() * linVelNed.norm());
    }

}  // namespace SensorModelISA

#endif  // SENSORS_ISA_MODEL_HPP
