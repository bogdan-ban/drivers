#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include "communication.h"
#include "serialib.h"

using namespace std;

class UartCommunication : public Communication
{
public:

	UartCommunication(char* port_name, int baud_rate);

	int start();

	uint8_t read_from_channel();

	void write_to_channel(const uint8_t buffer[],int size);

	void stop();

	int convert_to_bytes(uint8_t vec[2]);

	void read_all_data(const string opt);

private:

	char* port;

	int baudrate;

	serialib serial;
};

#endif