/*  
    This program is free software; you can redistribute it and/or 
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

#include "mpu9150_bitmap.h"
#include <stdint>


// define struct for MPU9150 sensor
typedef struct {
	int fd;
	unsigned char address;
	double acc_x;
	double acc_y;
	double acc_z;
	double gyr_x;
	double gyr_y;
	double gyr_z;
	double mag_x;
	double mag_y;
	double mag_z;
	float temp;
} t_mpu_9150;


// prototypes
int mpu9150_init(t_mpu9150 *);
int mpu9150_reset(t_mpu9150 *);
//int mpu9150_measure(t_ms5611 *);
//int mpu9150_calculate(t_ms5611 *);
int mpu9150_open(t_mpu9150 *);

//int ms5611_read_pressure(t_ms5611 *);
int mpu9150_read_temp(t_mpu9150 *);
int mpu9150_read_data(t_mpu9150 *);
int mpu9150_write(t_mpu9150 *, unsigned char, unsigned char);
//int ms5611_start_temp(t_ms5611 *);
//int ms5611_start_pressure(t_ms5611 *);
int mpu9150_verify_conn(t_mpu9150 *);
int mpu9150_verify_mag_conn(t_mpu9150 *);
int mpu9150_enable_mag_passthrough(t_mpu9150 *,bool);
int mpu9150_start_mag(t_mpu9150 *);
int mpu9150_read_mag(mpu9150 *);
int mpu9150_get_mag_asa(mpu9150 *);


