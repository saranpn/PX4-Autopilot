/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**

 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/integrated_accel.h>

__EXPORT int integ_accel_custom_module_main(int argc, char *argv[]);

int integ_accel_custom_module_main(int argc, char *argv[])
{

	vehicle_status_s vehicle_status;
	bool armed = true;
	do
	{
	    if (_vehicle_status_sub.update(&vehicle_status))
	    {

		   armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

		   if(armed)
		   {
			   //Get accel data
			   //sensor_accel topic
			   // Subscirbe to "sensor_accel", then set a polling interval of 200ms
				int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
				orb_set_interval(accel_sub, 200);

				// Configure a POSIX POLLIN system to sleep the current thread until
				// data appears on the topic
				px4_pollfd_struct_t fds_accel;
				fds_accel.fd = accel_sub;
				fds_accel.events = POLLIN;


				// Allow the POSIX POLLIN system to poll for data, with 1000ms timeout
				int poll_ret = px4_poll(&fds_accel, 1, 1000);

				// If px4_poll returns 0, then the poll system timed out! Throw an error.
				if(poll_ret == 0)
				{
				PX4_ERR("Got no data within a second");
				}

				// If it didn't return 0, we got data!
				else
				{
				// Double check that the data we recieved was in the right format (I think - need to check)
				if(fds_accel.revents & POLLIN)
				{

					// Create a sensor_accel_s struct to store the data we recieved
					struct sensor_accel_s accel;

					// Copy the data over to the struct
					orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);



			   //Publish to topic integ_accel
			   struct integrated_accel_s integrated_accel;

			   memset(&integrated_accel, 0, sizeof(integrated_accel));

			   orb_advert_t integ_accel_pub = orb_advertise(ORB_ID(integrated_accel),&integrated_accel);

			   //set integ accel struct with integ data
			   //integrated_accel.integrated_vale = found value

			   integrated_accel.vel_x += acccel.x * accel.integral_dt;
			   integrated_accel.vel_y += acccel.y * accel.integral_dt;
			   integrated_accel.vel_z += acccel.z * accel.integral_dt;


			   orb_publish(ORB_ID(integrated_accel), integ_accel_pub, &integrated_accel);

			   }
			}

		   }
	    }

	} while (armed);


	PX4_INFO("Exit integ accel custom module");
	return 0;
}
