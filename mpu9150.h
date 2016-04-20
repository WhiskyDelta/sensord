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
#include "mpu9150_register_map.h"
#include <stdint>


// define struct for MPU9150 sensor
typedef struct {
	int fd;
	int fd_mag;
	uint8_t address;
	uint8_t address_mag;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	int16_t asa_x;
	int16_t asa_y;
	int16_t asa_z;
	int16_t temp;
	bool present
} t_mpu_9150;


// prototypes
int mpu9150_init(t_mpu9150 *);
int mpu9150_reset(t_mpu9150 *);
int mpu9150_open(t_mpu9150 *);

int mpu9150_read_data(t_mpu9150 *);
bool mpu9150_connected(t_mpu9150 *);

int mpu9150_open_mag(t_mpu9150 *);
int mpu9150_init_mag(t_mpu9150 *);
int mpu9150_reset_mag(t_mpu9150 *);
bool mpu9150_mag_connected(t_mpu9150 *);
int mpu9150_start_mag(t_mpu9150 *);
int mpu9150_read_mag(t_mpu9150 *);
bool mpu9150_mag_data_ready(t_mpu9150 *);


