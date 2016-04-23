/*  sensorcal - Sensor Calibration for Openvario Sensorboard - http://www.openvario.org/
    Copyright (C) 2014  The openvario project
    A detailed list of copyright holders can be found in the file "AUTHORS" 

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

#include "24c16.h"
#include "ams5915.h"
#include "sensorcal.h"
#include "mpu9150.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
	
int g_debug=0;
FILE *fp_console=NULL;
	
int calibrate_ams5915(t_eeprom_data* data)
{
	t_ams5915 dynamic_sensor;
	float offset=0.0;
	int i;
	
	// open sensor for differential pressure
	/// @todo remove hardcoded i2c address for differential pressure
	printf("Open sensor ...");
	if (ams5915_open(&dynamic_sensor, 0x28) != 0)
	{
		printf(" failed !!\n");
		return 1;
	}
	else
	{
		printf(" success!\n");
	}
	
	dynamic_sensor.offset = 0.0;
	dynamic_sensor.linearity = 1.0;
	
	//initialize differential pressure sensor
	ams5915_init(&dynamic_sensor);
	
	for (i=0;i<10 ;i++)
	{
		// read AMS5915
		ams5915_measure(&dynamic_sensor);
		ams5915_calculate(&dynamic_sensor);
		
		// wait some time ...
		usleep(1000000);
		printf("Measured: %f\n",dynamic_sensor.p);
		
		// calc offset 
		offset += dynamic_sensor.p;
	}
		
	data->zero_offset = -1*(offset/10);
	
	return(0);
}

int calibrate_mag(float *dest1, float *dest2)
{
	uint16_t ii = 0, jj = 0, sample_count = 0;
	float mag_bias[3] = {0, 0, 0};
	float mag_scale[3] = {0, 0, 0};
	float mag_min[3] = {9999, 9999, 9999};
	float mag_max[3] = {-9999, -9999, -9999};
	float mag_temp[3] = {0, 0, 0};

	t_mpu9150 accel_sensor;

	// open acceleration, gyro sensor
	if (mpu9150_open(&accel_sensor) != 0)
	{
		fprintf(stderr, "Open sensor failed !!\n");
	}

	//initialize accel sensor
	mpu9150_init(&accel_sensor);

	// open mag sensor
	mpu9150_open_mag(&accel_sensor);

	// initialize mag sensor
	mpu9150_init_mag(&accel_sensor);

	memcpy(accel_sensor.hard_iron, (float[3]){0,0,0}, 3);
	memcpy(accel_sensor.soft_iron, (float[3]){1,1,1}, 3);

	printf("Mag Calibration: Wave device in a figure eight until done!");
	usleep(4000000);

	// start first mag measurement
	mpu9150_start_mag(&accel_sensor);
	usleep(125000); 

	sample_count = 128;
	for(ii = 0; ii < sample_count; ii++)
	{
		mpu9150_read_mag(&accel_sensor);  // Read the mag data   
		memcpy(mag_temp, accel_sensor.mag, 3);
		for (jj = 0; jj < 3; jj++) 
		{
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
	mpu9150_start_mag(&accel_sensor);
	usleep(125000);  // at 8 Hz ODR, new mag data is available every 125 ms
	}
	
	// hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	memcpy(dest1, mag_bias, 3);  // save mag biases in G for main program  

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

 	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 	avg_rad /= 3.0;

 	dest2[0] = avg_rad/((float)mag_scale[0]);
 	dest2[1] = avg_rad/((float)mag_scale[1]);
 	dest2[2] = avg_rad/((float)mag_scale[2]);

 	printf("Mag Calibration done!");
	return 0;
}

int main (int argc, char **argv) {
	
	// local variables
	t_24c16 eeprom;
	t_eeprom_data data;
	
	int exit_code=0;
	int result;
	int c;
	int i;
	char zero[1]={0x00};
	
	
	// usage message
	const char* Usage = "\n"\
	"  -c              calibrate sensors\n"\
	"  -s [serial]     write serial number (6 characters)\n"\
    "  -i              initialize EEPROM. All values will be cleared !!!\n"\
	"\n";
	
	// disable buffering 
	setbuf(stdout, NULL);
	
	// print banner
	printf("sensorcal V%c.%c RELEASE %c build: %s %s\n", VERSION_MAJOR, VERSION_MINOR, VERSION_RELEASE,  __DATE__, __TIME__);
	printf("sensorcal Copyright (C) 2014  see AUTHORS on www.openvario.org\n");
	printf("This program comes with ABSOLUTELY NO WARRANTY;\n");
	printf("This is free software, and you are welcome to redistribute it under certain conditions;\n"); 
	
	// open eeprom object
	result = eeprom_open(&eeprom, 0x50);
	if (result != 0)
	{
		printf("No EEPROM found !!\n");
		//exit(1);
	}
		
	// check commandline arguments
	while ((c = getopt (argc, argv, "his:cdem")) != -1)
	{
		switch (c) {
			case 'h':
				printf("Usage: sensorcal [OPTION]\n%s",Usage);
				break;
				
			case 'i':
				printf("Initialize EEPROM ...\n");
				for (i=0; i<128; i++)
				{
					result = eeprom_write(&eeprom, &zero[0], i, 1);
				}
				strcpy(data.header, "OV");
				data.data_version = EEPROM_DATA_VERSION;
				strcpy(data.serial, "000000");
				data.zero_offset=0.0;
				update_checksum(&data);
				printf("Writing data to EEPROM ...\n");
				result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
				break;
			
			case 'c':
				// read actual EEPROM values
				printf("Reading EEPROM values ...\n\n");
				if( eeprom_read_data(&eeprom, &data) == 0)
				{
					calibrate_ams5915(&data);
					printf("New Offset: %f\n",(data.zero_offset));
					update_checksum(&data);
					printf("Writing data to EEPROM ...\n");
					result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
				}
				else
				{
					printf("EEPROM content not valid !!\n");
					printf("Please use -i to initialize EEPROM !!\n");
					exit_code=2;
					break;
				}						
				break;
			case 'e':
				// delete complete EEPROM
				printf("Delete whole EEPROM ...\n\n");
				for (i=0; i< sizeof(data); i++)
				{
						result = eeprom_write(&eeprom, &zero[0], 0x00, 1);
				}
				printf("EEPROM cleared !!\n");
				exit_code=3;
				printf("End ...\n");
				exit(exit_code);
				break;
			case 'd':
				// read actual EEPROM values
				printf("Reading EEPROM values ...\n\n");
				if( eeprom_read_data(&eeprom, &data) == 0)
				{
					printf("Actual EEPROM values:\n");
					printf("---------------------\n");
					printf("Serial: \t\t\t%s\n", data.serial);
					printf("Differential pressure offset:\t%f\n",data.zero_offset);
				}
				else
				{
					printf("EEPROM content not valid !!\n");
					printf("Please use -i to initialize EEPROM !!\n");
					exit_code=2;
					break;
				}
				printf("End ...\n");
				exit(exit_code);
				break;
				
			case 's':
				if( strlen(optarg) == 6)
				{
					// read actual EEPROM values	
					if( eeprom_read_data(&eeprom, &data) == 0)
					{
						for(i=0; i<7;i++)
						{
							data.serial[i]=*optarg;
							optarg++;
						}
						data.serial[7]='\n';
						printf("New Serial number: %s\n",data.serial);
						update_checksum(&data);
						printf("Writing data to EEPROM ...\n");
						result = eeprom_write(&eeprom, (char*)&data, 0x00, sizeof(data));
					}
					else
					{
						printf("EEPROM content not valid !!\n");
						printf("Please use -i to initialize EEPROM !!\n");
						exit_code=2;
						break;
					}
				}
				else
				{
					printf("ERROR: Serialnumber has to have exactly 6 characters !!\n");
					exit_code=1;
					break;
				}
				break;

			case 'm': ;	//empty statement, because of c
				float hard_iron[3];
				float soft_iron[3];
				calibrate_mag(hard_iron, soft_iron);
				//TODO write calibration data in file
				int ii;
				for (ii = 0; ii < 3; ii++) 
					printf("New hard iron value %d: %fµT\n", ii, hard_iron[ii]);
				for (ii = 0; ii < 3; ii++) 
					printf("New soft iron value %d: %f\n", ii, soft_iron[ii]);
				break;
				
			case '?':
				printf("Unknow option %c\n", optopt);
				printf("Usage: sensorcal [OPTION]\n%s",Usage);
				printf("Exiting ...\n");
		}
	}
		
	printf("End ...\n");
	return(exit_code);
}
	
