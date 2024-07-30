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

#include "UavDynamics/math/geodetic.hpp"
#include <cmath>

// Constants
static const double a = 6378137.0;              // WGS-84 Earth semimajor axis (m)
static const double f = 1.0 / 298.257223563;    // WGS-84 flattening
static const double b = a * (1 - f);            // Semi-minor axis
static const double e_sq = f * (2 - f);         // Square of eccentricity


CoordinateConverter::CoordinateConverter() : ref_lat_deg(0), ref_lon_deg(0), ref_alt_meters(0) {}

void CoordinateConverter::setInitialValues(double lat_deg, double lon_deg, double alt_meters) {
    ref_lat_deg = lat_deg;
    ref_lon_deg = lon_deg;
    ref_alt_meters = alt_meters;
    
    // Precompute cos and sin of latitude and longitude for efficiency
    cos_lat = cos(degreesToRadians(lat_deg));
}

void CoordinateConverter::enuToGeodetic(
                double local_enu_x_meters, double local_enu_y_meters, double local_enu_z_meters,
                double* lat_deg, double* lon_deg, double* alt_meters) const {
    double dlat = (local_enu_y_meters / earthRadiusMeters()) * (180.0 / M_PI);
    double dlon = (local_enu_x_meters / (earthRadiusMeters() * cos_lat)) * (180.0 / M_PI);
    double dalt = local_enu_z_meters;

    *lat_deg = ref_lat_deg + dlat;
    *lon_deg = ref_lon_deg + dlon;
    *alt_meters = ref_alt_meters + dalt;
}

double CoordinateConverter::degreesToRadians(double degrees) const {
    return degrees * M_PI / 180.0;
}

double CoordinateConverter::earthRadiusMeters() const {
    return 6378137.0; // WGS-84 Earth radius in meters
}
