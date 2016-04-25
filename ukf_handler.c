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


int ukf_handler_init(t_ukf_handler *handler)
{
	handler->dt = 2000;
	handler->counter = 0;

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

int ukf_handler_set_measurements(t_ukf_handler *handler)
{
	//clear previous measurements
	ukf_sensor_clear();

	// do every time
	// get and set accelerometer and gyroscope data
	ukf_sensor_set_accelerometer(real_t x, real_t y, real_t z);
	ukf_sensor_set_gyroscope(real_t x, real_t y, real_t z)
	
	if (handler->counter%(10/handler->dt) == 0) {	//do every 10ms
		// get and set and restart magnetometer
		ukf_sensor_set_magnetometer(real_t x, real_t y, real_t z);
	}

	if (handler->counter%(125/handler->dt) == 0) {	//do every 125ms
		// get and calculate and set pressure data
		ukf_sensor_set_pitot_tas(real_t tas)
		ukf_sensor_set_barometer_amsl(real_t amsl)
	}
	
	if (handler->counter%(1000/handler->dt) == 0) {	//do every second
		//get and set gps data
		ukf_sensor_set_gps_position(real_t lat, real_t lon, real_t alt);
		ukf_sensor_set_gps_velocity(real_t x, real_t y, real_t z);
		//send nmea

		handler->counter = 0;
	}
	handler->counter++;
	return (0);
}

int ukf_handler_advance_timestep(t_ukf_handler *handler)
{
	//advance state in time
	//(get new state)
	//wait until desired dT
	gettimeofday(current, NULL);
	suseconds_t dt = handler->current.tv_usec - handler->start.tv_usec;
	int diff = dt - handler->dt;
	if (diff > 120) usleep(diff-120); //TODO replace with nanosleep(), usleep() will be removed with POSIX.1-2008
	gettimeofday(start, NULL);
	return (0);
}

//maybe implement state getter (propably not)
