/*
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

#ifndef SRC_MATH_GEODETIC_HPP
#define SRC_MATH_GEODETIC_HPP

class CoordinateConverter {
public:
    CoordinateConverter();

    void setInitialValues(double lat, double lon, double alt);

    void enuToGeodetic(double enu_x, double enu_y, double enu_z, double* lat, double* lon, double* alt) const;

private:
    // Helper method to convert degrees to radians
    double degreesToRadians(double degrees) const;

    // Approximate radius of the Earth in meters
    double earthRadiusMeters() const;

    // Initial geodetic coordinates

    double ref_lat_deg;
    double ref_lon_deg;
    double ref_alt_meters;

    // Precompute values for efficiency
    double cos_lat;
};

#endif  // SRC_MATH_GEODETIC_HPP
