#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "serialib.h"
#include <linux/i2c-dev.h>
#include "uart_communication.h"
#include "i2c_communication.h"
#include "communication.h"


using namespace std;


class DataCollector
{
private:
	ros::Publisher pub;
	/*int baud_rate;
	char* serial_port;
	char* i2c_port;
	*/

public:
	// serialib serial;

	Communication* communication;

	DataCollector(ros::NodeHandle *nh);

	/*int open_UART();

	void print_to_UART(const uint8_t buffer[],int size);

	uint8_t read_from_UART();

	void close_UART();

	int open_I2C();

	void print_to_I2C(int fd, uint8_t buffer[]);

	uint8_t read_from_I2C(int fd);

	void close_I2C(int bus);

	void flush_serial_receiver();

	void read_data_UART(const string opt);

	int convert_bytes_2_int(uint8_t vec[2]);

	void read_data_I2C(int bus, const string opt);
	*/
};

#endif