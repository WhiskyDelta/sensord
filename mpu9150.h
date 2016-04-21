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
#include <stdint.h>
#include <stdbool.h>


// define struct for MPU9150 sensor
typedef struct {
	int fd;
	int fd_mag;
	uint8_t address;
	uint8_t address_mag;
	float acc_x;
	float acc_y;
	float acc_z;
	float gyr_x;
	float gyr_y;
	float gyr_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float asa_x;
	float asa_y;
	float asa_z;
	uint8_t AFS_SEL;	//0: +-2g, 1: +-4g, 2:+-8g, 3:+-16g
	uint8_t FS_SEL;		//0: +-250°/s, 1: +-500°/s, 2: +-1000°/s, 3: +-2000°/s
	float temp;
	bool present;
} t_mpu9150;


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


