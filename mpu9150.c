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

#include "mpu9150.h"
#include <string.h>

/**
* @brief Establish connection to MPU9150 pressure sensor
* @param sensor pointer to sensor instance
* @return result
*
* @date 19.04.2016 created
*
*/ 
int mpu9150_open(t_mpu9150 *sensor)
{
	// local variables
	int fd;
	
	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);
	
	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, MPU9150_I2C_ADDRESS) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return (1);
	else sensor->address = MPU9150_I2C_ADDRESS;
	}
	if (!mpu9150_connected(sensor)) {
		if (ioctl(fd, I2C_SLAVE, MPU9150_I2C_ALT_ADDRESS) < 0) {
			fprintf(stderr, "ioctl error: %s\n", strerror(errno));
			return (1);
		else sensor->address = MPU9150_I2C_ALT_ADDRESS;
		}
	}

	if (g_debug > 0) printf("Opened mpu9150 on 0x%x\n", sensor->address);
	
	// assign file handle to sensor object
	sensor->fd = fd;
	return (0);
}

int mpu9150_init(t_mpu9150 *sensor)
{
	uint8_t buf[2];
	buf[0] = PWR_MGMT_1;		//address to write to
	buf[1] = 0b00000001;		//disable sleep, set clock reference to x-gyro (8MHz)
	if (mpu9150_write(sensor, buf, 2) != 2) {
		return (1);
	}

	buf[0] = INT_PIN_CFG;		//address to write to
	buf[1] = 0b00000010;		//enable i2c bypass to access magnetometer directly
	if (mpu9150_write(sensor, buf, 2) != 2) {
		return (1);
	}
	//TODO stuff
	return (0)
}

bool mpu9150_connected(t_mpu9150 *sensor)
{
	uint8_t buf[1];
	buf[0] = WHO_AM_I;							//first address to read from
	if (write(sensor->fd, buf, 1) != 1) {					//write address to read from
		printf("Error writing to i2c slave(%s)\n", __func__);
		return (FALSE);
	}
	if (read(sensor->fd, buf, 1) != 1) {					// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return (FALSE);
	}
	if (buf[0] ==  0x68) {							//register WHO_AM_I is always 0x68
		sensor->present = TRUE;
		return TRUE;
	}
	return FALSE;
}

int mpu9150_reset(t_mpu9150 *sensor)
{
	uint8_t buf[2];
	buf[0] = PWR_MGMT_1;		//address to write to
	buf[1] = 0b10000000;		//set reset bit
	if (mpu9150_write(sensor, buf, 2) != 2) {
		return (1);
	}
	return (0);
}

int mpu9150_read_data(t_mpu9150 *sensor)
{
	uint8_t buf[14];
	buf[0] = ACCEL_XOUT_H;							//first address to read from
	if ((write(sensor->fd, buf, 1)) != 1) {					//write address to read from
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	if (read(sensor->fd, buf, 14) != 14) {					// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return(1);
	}
	//TODO fuse bytes and write in struct
	return (0);
}

int mpu9150_open_mag(t_mpu9150 *sensor)
{
	// local variables
	int fd;
	
	// try to open I2C Bus
	fd = open("/dev/i2c-1", O_RDWR);
	
	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, AK8975_I2C_ADDRESS) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		return (1);
	else sensor->address_mag = AK8975_I2C_ADDRESS;
	}
	if (!mpu9150_mag_connected(sensor)) {
		return (1);
	}

	if (g_debug > 0) printf("Opened mpu9150 on 0x%x\n", sensor->address_mag);
	
	// assign file handle to sensor object
	sensor->fd_mag = fd;
	return (0);
}

int mpu9150_init_mag(t_mpu9150 *sensor)
{
	uint8_t buf[3];
	buf[0] = 0x03;								//address to write to, CNTL-register
	buf[1] = 0b00001111;							//byte to write, fuse rom access mode
	if ((write(sensor->fd_mag, buf, 2)) != 2) {
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = 0x10;								//write address to read from
	if ((write(sensor->fd_mag, buf, 1)) != 1) {
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	if (read(sensor->fd_mag, buf, 3) != 3) {				// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = 0x03;								//address to write to, CNTL-register
	buf[1] = 0b00000000;							//byte to write, power down mode
	if ((write(sensor->fd_mag, buf, 2)) != 2) {
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	//TODO check data validity
	//TODO fuse bytes and write in struct
	return (0);
}

int mpu9150_start_mag(t_mpu9150 *sensor)
{
	uint8_t buf[2];
	buf[0] = 0x03;								//address to write to, CNTL-register
	buf[1] = 0b00000001;							//byte to write, single measurement mode
	if ((write(sensor->fd_mag, buf, 2)) != 2) {
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	return (0);
}

int mpu9150_read_mag(t_mpu9150 *sensor)
{
	uint8_t buf[7];	

	// download data
	buf[0] = 0x03;								//first address to read from
	if ((write(sensor->fd_mag, buf, 1)) != 1) {				//write address to read from
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	if (read(sensor->fd_mag, buf, 7) != 7) {				// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return(1);
	}
	
	//check data validity
	if (buf[7] != 0b00000000) {
		return (2);							//Data error or magnetic sensor overflow
	}
	
	//TODO fuse bytes and write in struct
	return (0);
}

bool mpu9150_mag_data_ready(t_mpu9150 *sensor)
{
	uint8_t buf[1];
	buf[0] = 0x02;								//first address to read from
	if ((write(sensor->fd_mag, buf, 1)) != 1) {				//write address to read from
		printf("Error writing to i2c slave(%s)\n", __func__);
		return FALSE;
	}
	if (read(sensor->fd_mag, buf, 1) != 1) {				// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return FALSE;
	}
	if (buf[0] == 1) {							// Data ready bit is set
		return TRUE;
	}
	return FALSE;
}

