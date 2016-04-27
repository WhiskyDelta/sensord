
#include "mpu9150.h"
#include <unistd.h>
#include <stdio.h>

int g_debug = 2;

int main (int argc, char **argv) {
	t_mpu9150 accel;
	mpu9150_open(&accel);
	mpu9150_init(&accel);
	mpu9150_open_mag(&accel);
	mpu9150_init_mag(&accel);
	mpu9150_start_mag(&accel);

	usleep(10000);
	while(1) {
		mpu9150_read_data(&accel);
		mpu9150_read_mag(&accel);
		printf("\r\t%f\t%f\t%f\t|\t%f\t%f\t%f\t|\t%f\t%f\t%f\t",accel.acc[0],accel.acc[1],accel.acc[2],accel.gyr[0],accel.gyr[1],accel.gyr[2],accel.mag[0],accel.mag[1],accel.mag[2]);
		usleep(10000);
	}
	return 0;
}
	
