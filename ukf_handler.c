/*  This program is free software; you can redistribute it and/or 
    modify it under the terms of the GNU General Public License 
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include "ukf_handler.h"
#include "libcukf.so"		// https://github.com/sfwa/ukf/
#include <stdint.h>


int ukf_handler_init(t_ukf_handler *)
{
	//set dynamic model
	//TODO better custom model?
	ukf_choose_dynamics(UKF_MODEL_CENTRIPETAL);
	
	//set dynamics noise
	//TODO from file?
	float pro_cov[] = {
	1e-14, 1e-14, 1e-4,  // Position covariance: lat, lon, alt
	7e-5, 7e-5, 7e-5,    // Velocity covariance: n, e, d
	2e-4, 2e-4, 2e-4,    // Acceleration covariance: body x, y, z
	3e-9, 3e-9, 3e-9,    // Attitude covariance: yaw, pitch roll
	2e-3, 2e-3, 2e-3,    // Angular velocity covariance: x, y, z
	1e-3, 1e-3, 1e-3,    // Angular acceleration covariance: x, y, z
	1e-5, 1e-5, 1e-5,    // Wind velocity covariance: n, e, d
	3e-12, 3e-12, 3e-12  // Gyro bias covariance: x, y, z
	};
	ukf_set_process_noise(pro_cov);



	//set sensor model (position, orientation) and sensor noise
	//TODO from file?
	ukf_ioboard_params_t params = {
	{0,0,0,1},	//accel orientation (x, y, z, W)
	{0,0,0},	//accel offset (forward, starboard, down)
	{0,0,0,1},	//gyro orientation (x, y, z, W)
	{0,0,0,1},	//mag orientation (x, y, z, W)
	{0,0,0},	//magnetic field (north, east, down)[µT]
	{0.84*0.84, 0.84*0.84, 1.54*1.54},	//accel covariance [(m/s²)²]
	{10*10*3.1415*3.1415/(180*180),
	    10*10*3.1415*3.1415/(180*180),
	    10*10*3.1415*3.1415/(180*180)},	//gyro covariance [(rad/s)²]
	{1.5, 1.5, 1.5},	//mag covariance [µT²]
	{1e-12, 1e-12, 225.0},	//GPS position covariance [rad², rad², m²]
	{9.0, 9.0, 49.0},	//GPS velocity covariance [(m/s)²]
	100,	//pitot TAS covariance [(m/s)²]
	4	//baro AMSL covariance m²
	}
	ukf_set_params(params);
	
	//TODO set initial state
	return (0);
}

int ukf_handler_set_measurements(t_ukf_handler *)
{
	//clear previous measurements
	//set available measurements
	return (0);
}

int ukf_handler_advance_timestep(t_ukf_handler *,float)
{
	//wait until desired dT
	//advance state in time
	//(get new state)
	return (0);
}

//maybe implement state getter (propably not)
