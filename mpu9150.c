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
#include "def.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "def.h"

extern int g_debug;

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
	
	// assign file handle to sensor object
	sensor->fd = fd;

	if (fd < 0) {
		fprintf(stderr, "Error opening file: %s\n", strerror(errno));
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, MPU9150_I2C_ADDRESS) < 0) {
		fprintf(stderr, "ioctl error: %s\n", strerror(errno));
		sensor->present = 0;
		return (1);
	}
	else sensor->address = MPU9150_I2C_ADDRESS;
	
	if (!mpu9150_connected(sensor)) {
		if (ioctl(fd, I2C_SLAVE, MPU9150_I2C_ALT_ADDRESS) < 0) {
			fprintf(stderr, "ioctl error: %s\n", strerror(errno));
			sensor->present = 0;
			return (1);
		} else {
			sensor->address = MPU9150_I2C_ALT_ADDRESS;
			sensor->present = 1;
		}
	} else sensor->present = 1;

	if (g_debug > 0) printf("Opened mpu9150 on 0x%x\n", sensor->address);
	
	return (0);
}

int mpu9150_init(t_mpu9150 *sensor)
{
	//sensor->FS_SEL = 0;		//set gyro range to +-250°/s
	//sensor->AFS_SEL = 1;		//set acc range to +-4g


	uint8_t buf[2];
	buf[0] = PWR_MGMT_1;		//address to write to
	buf[1] = 0b00000001;		//disable sleep, set clock reference to x-gyro (8MHz)
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = INT_PIN_CFG;		//address to write to
	buf[1] = 0b00000010;		//enable i2c bypass to access magnetometer directly
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = CONFIG;		//address to write to
	buf[1] = 0b00000010;		//set lowpass to 94Hz and 98Hz for acc and gyro
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = GYRO_CONFIG;		//address to write to
	buf[1] = sensor->FS_SEL<<3;	//set gyro range
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	buf[0] = ACCEL_CONFIG;		//address to write to
	buf[1] = sensor->AFS_SEL<<3;	//set acc range
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	//TODO stuff
	return (0);
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
	if ((write(sensor->fd, buf, 2)) != 2) {					//write address and data byte
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
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

	//fuse bytes and write in struct
	int ii;	
	for (ii = 0; ii < 3; ii++)
	{
		sensor->acc[ii] = ((int16_t)((buf[ ii*2   ]<<8) + buf[(ii*2)+1])) * (sensor->AFS_SEL+1)/16384.0;	//m/s²
		sensor->gyr[ii] = ((int16_t)((buf[(ii*2)+8]<<8) + buf[(ii*2)+9])) * (sensor->FS_SEL+1)*M_PI/180.0/131.0;//rad/s
	}
	
			
	sensor->temp = ((buf[6]<<8) + buf[7])/230+35;				// °C
	sensor->new_data = TRUE;
	//sensor->acc_x = ((buf[0]<<8) + buf[1])*(sensor->AFS_SEL+1)/16384;
	//sensor->acc_y = ((buf[2]<<8) + buf[3])*(sensor->AFS_SEL+1)/16384;		// g
	//sensor->acc_z = ((buf[4]<<8) + buf[5])*(sensor->AFS_SEL+1)/16384;
	//sensor->gyr_x = ((buf[8]<<8) + buf[9])*(sensor->FS_SEL+1)/131;
	//sensor->gyr_y = ((buf[10]<<8) + buf[11])*(sensor->FS_SEL+1)/131;		// °/s
	//sensor->gyr_z = ((buf[12]<<8) + buf[13])*(sensor->FS_SEL+1)/131;		

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
	}
	else sensor->address_mag = AK8975_I2C_ADDRESS;
	
	//if (!mpu9150_mag_connected(sensor)) return (1);

	if (g_debug > 0) printf("Opened mpu9150 on 0x%x\n", sensor->address_mag);
	
	// assign file handle to sensor object
	sensor->fd_mag = fd;
	return (0);
}

bool mpu9150_mag_connected(t_mpu9150 *sensor)
{
	uint8_t buf[2];	

	buf[0] = 0x00;								//first address to read from
	if ((write(sensor->fd_mag, buf, 1)) != 1) {				//write address to read from
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}
	if (read(sensor->fd_mag, buf, 2) != 2) {				// Read back data into buf[]
		printf("Unable to read from slave(%s)\n", __func__);
		return(1);
	}
	if (buf[0] != 0x48) {
		printf("Not properly connected to magnetometer!\n");	
		return FALSE;
	}
	return TRUE;
}

int mpu9150_init_mag(t_mpu9150 *sensor)
{
	//read sensitivity adjustment values and save them
	uint8_t buf[3];
	buf[0] = 0x0A;								//address to write to, CNTL-register
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

	//write in struct, x and y swapped because of orientation 
	sensor->asa[1] = (buf[0]-128)/256.0 + 1;
	sensor->asa[0] = (buf[1]-128)/256.0 + 1;
	sensor->asa[2] = (buf[2]-128)/256.0 + 1;

	buf[0] = 0x0A;								//address to write to, CNTL-register
	buf[1] = 0b00000000;							//byte to write, power down mode
	if ((write(sensor->fd_mag, buf, 2)) != 2) {
		printf("Error writing to i2c slave(%s)\n", __func__);
		return(1);
	}

	//TODO read calibration values from file
	float hard_init[3] = {101.636719,-73.886719,133.712891};
	float soft_init[3] = {0.938030,1.132728,0.951374};
	memcpy(sensor->hard_iron, hard_init, 3*sizeof(float));
	memcpy(sensor->soft_iron, soft_init, 3*sizeof(float));

	return (0);
}

int mpu9150_start_mag(t_mpu9150 *sensor)
{
	uint8_t buf[2];
	buf[0] = 0x0A;								//address to write to, CNTL-register
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
	if (buf[6] != 0b00000000) {
		return (2);							//Data error or magnetic sensor overflow
	}
	
	//fuse bytes and write in struct, order changed because orientation of
	//mag differs to acc and gyro orientation
	sensor->mag[1] =  (( ((int16_t)(buf[0]+(buf[1]<<8))) *sensor->asa[1]) + sensor->hard_iron[1]) * sensor->soft_iron[1];
	sensor->mag[0] =  (( ((int16_t)(buf[2]+(buf[3]<<8))) *sensor->asa[0]) + sensor->hard_iron[0]) * sensor->soft_iron[0];
	sensor->mag[2] = -(( ((int16_t)(buf[4]+(buf[5]<<8))) *sensor->asa[2]) + sensor->hard_iron[2]) * sensor->soft_iron[2];
	sensor->new_mag_data = TRUE;
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

