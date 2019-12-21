#include "model.h"
#include "haptlib.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

using namespace la;


/// structure pour passer les informations de retour de force via le port-série
struct ForceFeedback {
	enum Type {
		UNKNOWN,
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
		else if (strcmp(recvbuff, "none") == 0)		feedback.type = ForceFeedback::NONE;
		else										feedback.type = ForceFeedback::UNKNOWN;
		if (feedback.type == ForceFeedback::NONE)
			return feedback;
		
		size_t index = 0;
		for (size_t i=0; i<8; i++) {
			if (sscanf(recvbuff+index, "%f,", &feedback.vec(i)) != 1) {
				feedback.type = ForceFeedback::UNKNOWN;
				break;
			}
			while (recvbuff[index] != ',' && index < recvsize)	index++;
		}
	}
	else	feedback.type = ForceFeedback::UNKNOWN;
	return feedback;
}


// drapeaux d'activation des fonctionnalités
bool enable_feedback = true;	// retour de force
bool enable_measure = true;		// envoi de la pose sur le port serie
bool enable_assist = false;		// suivi de mouvement utilisateur (spasmes)

HaptikDXL dxl;
Delta model;
vec8 last_pose;
ForceFeedback feedback;
vec8 feedback_dir;
vec8 feedback_origin;

// nbr de périodes de loop entre chaque communications (envoi de pose et reception d'ordre)
static const int comrefresh_sample = 10;
int comrefresh = 0;	// compteur periodes depuis dernier envoi

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
	
	feedback = ForceFeedback {ForceFeedback::NONE, vec8(0.)};
	feedback_dir = vec8(0.);
	feedback_origin = last_pose;
}

void loop() {
	// restreindre les plages des moteurs (en attendant de faire ca avec des detections de singularités)
	const float force_current = 1.;	// (Nm/mA)	TODO: a determiner experimentalement
	const float current_corr = 5.;	// (mA/mm)	TODO: a affiner
	
	const float max_angle = 2;	// rad
	const float min_angle = -0.5;	// rad
	const float resist_current = 50;	// mA
	vec8 angle;
	
	// get the pose
	for (size_t i=0; i<N; i++) 	angle(i) = dxl.get_position(i);
	
	Delta::state pose = model.mgd_solve(angle, last_pose);	// compute the pose
	
	if (comrefresh == 0) {
		if (enable_measure) 
			send_pose(pose.X);	// send to computer
		
		if (enable_feedback) {
			// receive order and setup execution datas
			ForceFeedback new_fb = feedback = receive_feedback();
			if (new_fb.type != ForceFeedback::UNKNOWN)	{
				feedback = new_fb;
				switch (feedback.type) {
					case ForceFeedback::FORCE:
						feedback_dir = force_current * (model.mci(pose) * feedback.vec);
						break;
					case ForceFeedback::BLOCK:
						// le vecteur passé est une direction (donc vecteur normé), si sa norme n'est pas 1, elle servira de facteur a l'asservissement
						feedback_dir = current_corr * (model.mci(pose) * feedback.vec);
						feedback_origin = pose.X;
						break;
				}
			}
		}
	}
	comrefresh = (comrefresh+1)%comrefresh_sample;
	
	// apply feedback
	vec8 current;
	switch (feedback.type) {
		case ForceFeedback::FORCE:
			current = feedback_dir;
			break;
		case ForceFeedback::BLOCK:
			current = dot(pose.X - feedback_origin, feedback.vec) * feedback_dir;
			break;
		default:
			current = vec8(0.);
	}
	
	// apply limitations
	for (size_t i=0; i<N; i++) {
		if 		(angle(i) < min_angle)	current(i) += resist_current;
		else if (angle(i) > max_angle)	current(i) -= resist_current;
	}
	
	// apply torques to motors
	for (size_t i=0; i<N; i++)	dxl.set_current(i, current(i));
}
