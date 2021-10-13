#include "uart_communication.h"


UartCommunication::UartCommunication(char* port_name, int baud_rate)
{
	// port = (char*)"/dev/ttyTHS1";
	port = port_name;
	baudrate = baud_rate;
}

int UartCommunication::start()
{
	cout << "serial_port: " << port << endl;
	cout << "baud_rate: " << baudrate << endl;
	char errorOpening = serial.openDevice(port, baudrate);

    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1)
    	return errorOpening;
	cout << "Successful connection to " << port << endl;
}

uint8_t UartCommunication::read_from_channel()
{
	uint8_t bte[3] = {0x00};
	bool bte_flag = false;

	if(serial.available())
	{
		serial.readBytes(bte,serial.available());
		bte_flag = true;

		if(bte[0] == 0xBB)
		{
			return bte[2];
		}else 
			if(bte[0] == 0xEE)
			{
				printf("Error status: %x\n",bte[1]);
				return 0x00;
			}
	}
	else
	{
		cout<<"Error reading\n";
		return 0x00;
	}

	bte_flag = false;
	return 0x00;
}

void UartCommunication::write_to_channel(const uint8_t buffer[],int size)
{
	serial.writeBytes(buffer,size);
}

void UartCommunication::stop()
{
	try
	{
		serial.closeDevice();
	}
	catch(const std::exception& e)
	{
		cout << "Uart connection not ended. \"close()\" function was called" << endl;	
	}
}

int UartCommunication::convert_to_bytes(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}

void UartCommunication::read_all_data(const string opt)
{
	uint8_t read_DATA_X_MSB[4] = { '\xAA', '\x01', '\x0', '\x01'};
	uint8_t read_DATA_X_LSB[4] = { '\xAA', '\x01', '\x0', '\x01'};
	uint8_t read_DATA_Y_MSB[4] = { '\xAA', '\x01', '\x0', '\x01'};
	uint8_t read_DATA_Y_LSB[4] = { '\xAA', '\x01', '\x0', '\x01'};
	uint8_t read_DATA_Z_MSB[4] = { '\xAA', '\x01', '\x0', '\x01'};
	uint8_t read_DATA_Z_LSB[4] = { '\xAA', '\x01', '\x0', '\x01'};

	// for quaternion
	uint8_t read_DATA_W_MSB[4] = {'\xAA', '\x01', '\x21', '\x01'};
	uint8_t read_DATA_W_LSB[4] = {'\xAA', '\x01', '\x20', '\x01'};

	if(opt == "ACC")
	{
		read_DATA_X_MSB[2] = '\x09';
		read_DATA_X_LSB[2] = '\x08';
		read_DATA_Y_MSB[2] = '\x0B';
		read_DATA_Y_LSB[2] = '\x0A';
		read_DATA_Z_MSB[2] = '\x0D';
		read_DATA_Z_LSB[2] = '\x0C';
		printf("ACC: \n");
	}else
		if(opt == "MAG")
		{
			read_DATA_X_MSB[2] = '\x0F';
			read_DATA_X_LSB[2] = '\x0E';
			read_DATA_Y_MSB[2] = '\x11';
			read_DATA_Y_LSB[2] = '\x10';
			read_DATA_Z_MSB[2] = '\x13';
			read_DATA_Z_LSB[2] = '\x12';
			printf("MAG: \n");
		}else
			if(opt == "GYR")
			{
				read_DATA_X_MSB[2] = '\x15';
				read_DATA_X_LSB[2] = '\x14';
				read_DATA_Y_MSB[2] = '\x17';
				read_DATA_Y_LSB[2] = '\x16';
				read_DATA_Z_MSB[2] = '\x19';
				read_DATA_Z_LSB[2] = '\x18';
				printf("GYR: \n");
			}else
				if(opt == "EUL")
				{
					read_DATA_X_MSB[2] = '\x1B';
					read_DATA_X_LSB[2] = '\x1A';
					read_DATA_Y_MSB[2] = '\x1D';
					read_DATA_Y_LSB[2] = '\x1C';
					read_DATA_Z_MSB[2] = '\x1F';
					read_DATA_Z_LSB[2] = '\x1E';
					printf("EUL: \n");
				}else
					if(opt == "QUA")
					{
						read_DATA_X_MSB[2] = '\x23';
						read_DATA_X_LSB[2] = '\x22';
						read_DATA_Y_MSB[2] = '\x25';
						read_DATA_Y_LSB[2] = '\x24';
						read_DATA_Z_MSB[2] = '\x27';
						read_DATA_Z_LSB[2] = '\x26';
						printf("QUA: \n");
						// w
						uint8_t vec[2];
						write_to_channel(read_DATA_W_MSB,4);
						vec[1] = read_from_channel();
						//printf("LSB: %x\n", vec[1]);
						ros::Duration(0.1).sleep();

						write_to_channel(read_DATA_W_LSB,4);
						vec[0] = read_from_channel();
						//printf("MSB: %x\n", vec[0]);
						ros::Duration(0.1).sleep();

						printf("w: %d\n",convert_to_bytes(vec));
					}else
						if(opt == "LIA")
						{
							read_DATA_X_MSB[2] = '\x29';
							read_DATA_X_LSB[2] = '\x28';
							read_DATA_Y_MSB[2] = '\x2B';
							read_DATA_Y_LSB[2] = '\x2A';
							read_DATA_Z_MSB[2] = '\x2D';
							read_DATA_Z_LSB[2] = '\x2C';
							printf("LIA: \n");
						}else
							if(opt == "GRV")
							{
								read_DATA_X_MSB[2] = '\x2F';
								read_DATA_X_LSB[2] = '\x2E';
								read_DATA_Y_MSB[2] = '\x31';
								read_DATA_Y_LSB[2] = '\x30';
								read_DATA_Z_MSB[2] = '\x33';
								read_DATA_Z_LSB[2] = '\x32';
								printf("GRV: \n");
							}
		
	uint8_t vec[2];

	// x
	write_to_channel(read_DATA_X_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_X_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("x: %d\n",convert_to_bytes(vec));

	// y
	write_to_channel(read_DATA_Y_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_Y_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("y: %d\n",convert_to_bytes(vec));

	// z
	write_to_channel(read_DATA_Z_MSB,4);
	vec[1] = read_from_channel();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	write_to_channel(read_DATA_Z_LSB,4);
	vec[0] = read_from_channel();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("z: %d\n\n",convert_to_bytes(vec));
	ros::Duration(1).sleep();
}