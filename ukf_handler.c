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
#include "cukf.h"		// https://github.com/sfwa/ukf/
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include "def.h"


int ukf_handler_init(t_ukf_handler *handler, t_mpu9150 *accel_sensor, t_ams5915 *dynamic_sensor, t_ms5611 *static_sensor)
{
	handler->dt = 2000;
	handler->counter = 0;
	handler->QNH = 1013.25;

	//set dynamic model
	//TODO better custom model?
	ukf_choose_dynamics(UKF_MODEL_CENTRIPETAL);
	
	//set dynamics noise
	//TODO from file?
	real_t pro_cov[] = {
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
	struct ukf_ioboard_params_t params = {
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
	};
	ukf_set_params(&params);
	
	//TODO set initial state
	struct ukf_state_t intial = {
	{0,0,0},	//position, from gps
	{0,0,0},	//velocity, 0
	{0,0,0},	//acceleration, 0
	{0,0,0,1},	//attitude, to be calculated from mag
	{0,0,0},	//angular velocity, 0
	{0,0,0},	//angular acceleration, 0
	{0,0,0},	//wind velocity, 0
	{0,0,0}};	//gyro bias, gyro reading while standing still
	
	memcpy(initial.gyro_bias, accel_sensor->gyr, 3*sizeof(float));
	ukf_set_state(&initial);

	
	//QNH = p * e^(h*g/(R*288.15K))
	return (0);
}

int ukf_handler_set_measurements(t_ukf_handler *handler, t_mpu9150 *accel_sensor, t_ams5915 *dynamic_sensor, t_ms5611 *static_sensor)
{
	//clear previous measurements
	ukf_sensor_clear();

	// do every time
	// get and set accelerometer and gyroscope data
	//TODO check if data in correct unit
	ukf_sensor_set_accelerometer((real_t)accel_sensor->acc[0],(real_t)accel_sensor->acc[1],(real_t)accel_sensor->acc[2]);
	ukf_sensor_set_gyroscope((real_t)accel_sensor->gyr[0],(real_t)accel_sensor->gyr[1],(real_t)accel_sensor->gyr[2]);
	
	if (handler->counter%(10/handler->dt) == 0) {	//do every 10ms
		// get and set and restart magnetometer
		ukf_sensor_set_magnetometer((real_t)accel_sensor->mag[0],(real_t)accel_sensor->mag[1],(real_t)accel_sensor->mag[2]);
	}

	if (handler->counter%(125/handler->dt) == 0) {	//do every 125ms
		// get and calculate and set pressure data
		//TAS = sqrt(2 * q / ρ), ρ = p/RT 
		ukf_sensor_set_pitot_tas((real_t)( sqrt(2 * dynamic_sensor->p * R_AIR * dynamic_sensor->T / static_sensor->p) ));
		//AMSL = (R * 288.15K / g) * ln(QNH / p)
		ukf_sensor_set_barometer_amsl((real_t)( (R_AIR * 288.15 / 9.81) * (log(handler->QNH / static_sensor->p)) ));
	}
	
	if (handler->counter%(1000/handler->dt) == 0) {	//do every second
		//get and set gps data
		//ukf_sensor_set_gps_position(real_t lat, real_t lon, real_t alt);
		//ukf_sensor_set_gps_velocity(real_t x, real_t y, real_t z);
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
	gettimeofday(&handler->current, NULL);
	suseconds_t dt = handler->current.tv_usec - handler->start.tv_usec;
	int diff = dt - handler->dt;
	if (diff > 120) {
		usleep(diff - 120); //TODO replace with nanosleep(), usleep() will be removed with POSIX.1-2008
	}
	gettimeofday(&handler->start, NULL);
	return (0);
}

//maybe implement state getter (propably not)

