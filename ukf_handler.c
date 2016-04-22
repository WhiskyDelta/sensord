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


int ukf_handler_init(t_ukf_handler *)
{
	//set dynamic model
	//set dynamics noise
	//set sensor model (position, orientation)
	//set sensor noise
	//set initial state
	return (0);
}

int ukf_handler_set_measurements(t_ukf_handler *)
{
	//clear previous measurements
	//set available measurements
	return (0);
}

int ukf_handler_advance_timestep(t_ukf_handler *,float)
{
	//wait until desired dT
	//advance state in time
	//(get new state)
	return (0);
}

//maybe implement state getter (propably not)
