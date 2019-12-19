#include "model.h"
#include "haptlib.h"
#include <Arduino.h>
#include <stdio.h>

using namespace la;

HaptikDXL dxl;
Delta model;
vec8 last_pose;

/// send the pose through Serial
void send_pose(const vec8 & pose) {
	Serial.print("pose ");
	for (size_t i=0; i<N; i++) {
		Serial.print(pose(i));
		Serial.print(',');
	}
}



void setup() {
	Serial.begin(57600);
	dxl.begin("3", 1000000);
	
	for (int i=0; i<N; i++) {
		// check the motor connection
		if (!dxl.ping(i)) {
			Serial.print("motor ");
			Serial.print(i);
			Serial.println(" not responding");
		}
		
		// setup motor
		dxl.set_profile_acceleration(i, 30000.);
		dxl.set_profile_velocity(i, 200.);
		dxl.enable(i, true);
		
		// set the initial pose (in real and in memory)
		const float default_position = 0;
		dxl.set_mode(i, HaptikDXL::POSITION);
		dxl.set_position(i, default_position);
		last_pose = vec8(0.);
		delay(1000);
		
		// setup current for the rest of the program
		dxl.set_mode(i, HaptikDXL::CURRENT);
	}
}

void loop() {
	// restreindre les plages des moteurs (en attendant de faire ca avec des detections de singularités)
	const float max_angle = 2;	// rad
	const float min_angle = -0.5;	// rad
	const float resist_current = 50;	// mA
	vec8 angle;
	vec8 current = vec8(0.);
	
	// get the pose and apply limitations
	for (size_t i=0; i<N; i++) 	angle(i) = dxl.get_position(i);
	for (size_t i=0; i<N; i++) {
		if (angle(i) < min_angle)	current(i) = resist_current;
		if (angle(i) > max_angle)	current(i) = -resist_current;
	}
	
	vec8 pose = model.mgd_solve(angle, last_pose);	// compute the pose
	send_pose(pose);	// send to computer
	
	// apply torques to motors
	for (size_t i=0; i<N; i++)	dxl.set_current(i, current(i));
}
