#include <dynamixel_sdk/dynamixel_sdk.h>

class Bus {
public:
	dynamixel::PortHandler	port;
	dynamixel::PacketHandler protocol;
	
	
};

class Motor {
public:
	const Bus * bus;
	const uint8_t id;
	
	Motor(Bus* bus, uint8_t id)		:bus(bus), id(id)	{}
	
	set_reg_current(uint16_t p, uint16_t i);
	set_reg_velocity(uint16_t p, uint16_t i);
	set_reg_position(uint16_t p, uint16_t i);
	
	go_current(int16_t target);
	go_velocity(int16_t target);
	go_position(int16_t target);
	
	int16_t ac_current();
	int16_t ac_velocity();
	int16_t ac_position();
};

