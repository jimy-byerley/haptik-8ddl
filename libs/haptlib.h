#ifndef _haptlib_h
#define _haptlib_h

#include <DynamixelWorkbench.h>

//class HaptikDXL : public DynamixelDriver {
class HaptikDXL : public DynamixelWorkbench {
public:	
	
	enum DXLREG {
		MODE = 11,
		ENABLE = 64,
		
		PROFILE_ACCELERATION = 108,
		PROFILE_VELOCITY = 112,
		
		MIN_POSITION = 52,
		MAX_POSITION = 48,
		MAX_VOLTAGE = 36,
		
		GOAL_POSITION = 116,
		GOAL_CURRENT = 102,
		GOAL_VELOCITY = 104,
		GOAL_VOLTAGE = 100,
		
		VELOCITY_GAIN_I = 76,
		VELOCITY_GAIN_P = 78,
		
		POSITION_GAIN_D = 80,
		POSITION_GAIN_I = 82,
		POSITION_GAIN_P = 64,
		
		PRESENT_CURRENT = 126,
		PRESENT_VELOCITY = 128,
		PRESENT_POSITION = 132,
		PRESENT_VOLTAGE = 124
	};

	enum DXL_MODE {
		CURRENT = 0,
		VELOCITY = 1,
		POSITION = 3,
		VOLTAGE = 16
	};
	
	typedef uint16_t dxlid;

	const int32_t MAX_ENCODER = 4095;
	const float UNIT_ANGLE = 2*M_PI/MAX_ENCODER;
	const float UNIT_ACCELERATION = 214.577;
	const float UNIT_VELOCITY = 0.229;
	const float UNIT_CURRENT = 2.69;
	const float UNIT_VOLTAGE = 0.00113;
	
	/*
		recuperation d'informations
	*/
	// torque
	float get_torque(dxlid id) {
		int8_t _torque;
		readRegister(id, DXLREG::ENABLE, 1, (uint32_t*) &_torque);
		return _torque;
	}
	
	// courant en mA
	float get_current(dxlid id) {
		int16_t _current;
		readRegister(id, DXLREG::PRESENT_CURRENT, 2, (uint32_t*) &_current);
		return _current * UNIT_CURRENT;
	}
	
	// vitesse en rad/s
	float get_velocity(dxlid id) {
		int32_t _velocity;
		readRegister(id, DXLREG::PRESENT_VELOCITY, 4, (uint32_t*) &_velocity);
		return _velocity * UNIT_VELOCITY;
	}
	
	// position en rad
	float get_position(dxlid id) {
		int32_t _position;
		readRegister(id, DXLREG::PRESENT_POSITION, 4, (uint32_t*) &_position);
		return _position * UNIT_ANGLE;
	}
	
	// voltage
	float get_voltage(dxlid id) {
		int16_t _voltage;
		readRegister(id, DXLREG::PRESENT_VOLTAGE, 2, (uint32_t*) &_voltage);
		return _voltage * UNIT_VOLTAGE;
	}

	/* 
		controle moteur
		Attention, tant qu'on n'est pas dans le bon mode, les ordres ci dessous ne seront pas appliqués
	*/
	
	void enable(dxlid id, bool enable) {
		writeRegister(id, DXLREG::ENABLE, 1, (uint8_t*) &enable);
	}
	
	void set_mode(dxlid id, uint8_t mode) {
		enable(id, false);
		writeRegister(id, DXLREG::MODE, 1, (uint8_t*) &mode);
		enable(id, true);
	}
	
	/// voltage comme fraction de l'alim (1 pour max)
	void set_voltage(dxlid id, float voltage) {
		int16_t _voltage = voltage / UNIT_VOLTAGE;
		writeRegister(id, DXLREG::GOAL_VOLTAGE, 2, (uint8_t*) &_voltage);
		Serial.print("voltage set to");
		Serial.println(_voltage);
	}

	/// courant en mA
	void set_current(dxlid id, float current) {
		int16_t _current = current / UNIT_CURRENT;
		//set_mode(id, DXL_MODE::CURRENT);
		writeRegister(id, DXLREG::GOAL_CURRENT, 2, (uint8_t*) &_current);
	}
	
	/// position en rad
	void set_position(dxlid id, float position) {
		int32_t _position = position / UNIT_ANGLE;
		//set_mode(id, DXL_MODE::POSITION);
		writeRegister(id, DXLREG::GOAL_POSITION, 4, (uint8_t*) &_position);
	}
	
	/// velocity en rad
	void set_velocity(dxlid id, float velocity) {
		int32_t _velocity = velocity / UNIT_VELOCITY;
		//set_mode(id, DXL_MODE::POSITION);
		writeRegister(id, DXLREG::GOAL_VELOCITY, 4, (uint8_t*) &_velocity);
	}
	
	/*
		parametrage
	*/

	/// acceleration cible en rad/s2
	/// ce parametre sert aux asservissements de position et vitesse
	void set_profile_acceleration(dxlid id, float accel) {
		uint32_t _accel = accel / UNIT_ACCELERATION;
		writeRegister(id, DXLREG::PROFILE_ACCELERATION, 4, (uint8_t*) &_accel);
	}
	
	/// vitesse cible en rad/s
	/// ce parametre sert uniquement a l'asservissement de position
	void set_profile_velocity(dxlid id, float velocity) {
		uint32_t _velocity = velocity / UNIT_VELOCITY;
		writeRegister(id, DXLREG::PROFILE_VELOCITY, 4, (uint8_t*) &_velocity);
	}
	
	///position min (0 - 4095)
	void set_min_position(dxlid id, int32_t min) {
		writeRegister(id, DXLREG::MIN_POSITION, 4,(uint8_t*) &min);
	}
	
	///position max (0 - 4095)
	void set_max_position(dxlid id, int32_t max) {
		writeRegister(id, DXLREG::MAX_POSITION, 4,(uint8_t*) &max);
	}
	
	void set_max_voltage(dxlid id, float max) {
		uint16_t _voltage = 885; //max / UNIT_VOLTAGE;
		writeRegister(id, DXLREG::MAX_VOLTAGE, 2, (uint8_t*) &_voltage);
	}
	
};

#endif
