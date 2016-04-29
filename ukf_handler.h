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

//define struct

#include <sys/time.h>
#include "ms5611.h"
#include "ams5915.h"
#include "mpu9150.h"
#include "ads1110.h"
#include "cukf.h"

typedef struct {
	int counter;
	struct timeval start;
	struct timeval current;
	int dt; //mikroseconds
	float QNH;
	char message[2048];
	int message_length;
	int sock_err;
	struct ukf_state_t state;
} t_ukf_handler;

//prototypes
int ukf_handler_init(t_ukf_handler *, t_mpu9150 *, t_ams5915 *, t_ms5611 *);
int ukf_handler(t_ukf_handler *, t_mpu9150 *, t_ams5915 *, t_ms5611 *, t_ads1110 *, int(*sendFunction)(char*, int));

