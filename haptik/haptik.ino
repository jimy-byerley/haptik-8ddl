#include "model.h"
#include "haptlib.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

using namespace la;


struct ForceFeedback {
	enum Type {
		NONE,
		FORCE,
		BLOCK
	};
	
	Type type;
	vec8 vec;
};

/// send the pose through Serial
void send_pose(const vec8 & pose) {
	Serial.print("pose ");
	for (size_t i=0; i<N; i++) {
		Serial.print(pose(i));
		Serial.print(',');
	}
}

static const size_t recvsize = 256;
char recvbuff[recvsize];
/// receive characters in recvbuff until the endseq sequence is found (the ending sequence will be in recvbuff too)
bool receive(const char * endseq, const size_t endlen) {
	size_t recvindex = 0;
	size_t endindex = 0;
	while (Serial.available() && recvindex < recvsize) {
		char recv = recvbuff[recvindex++] = Serial.read();
		if (recv == endseq[endindex]) {
			endindex++;
			if (endindex >= endlen)		break;
		}
	}
	recvbuff[recvindex] = 0;
	return recvindex < recvsize && endindex == endlen;
}
/// receive a force-feedback info though the serial port
ForceFeedback receive_feedback() {
	ForceFeedback feedback;
	if (receive("\n", 1)) {
		if 		(strcmp(recvbuff, "force") == 0) 	feedback.type = ForceFeedback::FORCE;
		else if (strcmp(recvbuff, "block") == 0) 	feedback.type = ForceFeedback::BLOCK;
		else										feedback.type = ForceFeedback::NONE;
		size_t index = 0;
		for (size_t i=0; i<8; i++) {
			if (sscanf(recvbuff+index, "%f,", &feedback.vec(i)) != 1) {
				feedback.type = ForceFeedback::NONE;
				break;
			}
			while (recvbuff[index] != ',' && index < recvsize)	index++;
		}
	}
	else	feedback.type = ForceFeedback::NONE;
	return feedback;
}


bool enable_feedback = true;
bool enable_measure = true;
bool enable_assist = false;

HaptikDXL dxl;
Delta model;
vec8 last_pose;

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
		
		// setup current mode for the rest of the program
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
		if 		(angle(i) < min_angle)	current(i) = resist_current;
		else if (angle(i) > max_angle)	current(i) = -resist_current;
	}
	
	Delta::state pose = model.mgd_solve(angle, last_pose);	// compute the pose
	send_pose(pose.X);	// send to computer
	
	if (enable_feedback) {
		// receive order
		ForceFeedback feedback = receive_feedback();
		switch (feedback.type) {
			case ForceFeedback::FORCE:
				break;
			case ForceFeedback::BLOCK:	
				break;
		}
	}
	
	// apply torques to motors
	for (size_t i=0; i<N; i++)	dxl.set_current(i, current(i));
}
