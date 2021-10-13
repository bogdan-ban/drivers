#include "data_collector.h"


DataCollector::DataCollector(ros::NodeHandle *nh)
{
	// pub = nh->advertise<>("/collect_data",10);
	//nh->getParam("/baud_rate",baud_rate);
	//nh->getParam("/serial_port",serial_port);
	//nh->getParam("/i2c_port",i2c_port);
	
	//baud_rate = 115200;
	//serial_port = (char*)"/dev/ttyTHS1";
	//i2c_port = (char*)"/dev/i2c-1";
}

/*
int DataCollector::open_UART()
{
	// cout << "serial_port: " << serial_port.c_str() << endl;
	cout << "serial_port: " << serial_port << endl;
	cout << "baud_rate: " << baud_rate << endl;
	char errorOpening = serial.openDevice(serial_port, baud_rate);

    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1)
    	return errorOpening;
	cout << "Successful connection to " << serial_port << endl;
}

uint8_t DataCollector::read_from_UART()
{
	uint8_t bte[3] = {0x00};
	bool bte_flag = false;

	// ROS_INFO("How many bytes are available? %d\n",serial.available());

	if(serial.available())
	{
		serial.readBytes(bte,serial.available());
		bte_flag = true;

		if(bte[0] == 0xBB)
		{
			// printf("%x\n",bte[2]);
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

void DataCollector::print_to_UART(const uint8_t buffer[], int size)
{
	// for(int i=0;i<size;i++)
	//	 printf("%x ",buffer[i]);
	// cout << endl;
    serial.writeBytes(buffer,size);
    // cout << sizeof(buffer) << endl;
}

int DataCollector::convert_bytes_2_int(uint8_t vec[2])
{
	return 256 * vec[0] + vec[1];
}


void DataCollector::read_data_UART(const string opt)
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
						print_to_UART(read_DATA_W_MSB,4);
						vec[1] = read_from_UART();
						//printf("LSB: %x\n", vec[1]);
						ros::Duration(0.1).sleep();

						print_to_UART(read_DATA_W_LSB,4);
						vec[0] = read_from_UART();
						//printf("MSB: %x\n", vec[0]);
						ros::Duration(0.1).sleep();

						printf("w: %d\n",convert_bytes_2_int(vec));
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
	print_to_UART(read_DATA_X_MSB,4);
	vec[1] = read_from_UART();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	print_to_UART(read_DATA_X_LSB,4);
	vec[0] = read_from_UART();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("x: %d\n",convert_bytes_2_int(vec));

	// y
	print_to_UART(read_DATA_Y_MSB,4);
	vec[1] = read_from_UART();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	print_to_UART(read_DATA_Y_LSB,4);
	vec[0] = read_from_UART();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("y: %d\n",convert_bytes_2_int(vec));

	// z
	print_to_UART(read_DATA_Z_MSB,4);
	vec[1] = read_from_UART();
	//printf("LSB: %x\n", vec[1]);
	ros::Duration(0.1).sleep();

	print_to_UART(read_DATA_Z_LSB,4);
	vec[0] = read_from_UART();
	//printf("MSB: %x\n", vec[0]);
	ros::Duration(0.1).sleep();

	printf("z: %d\n\n",convert_bytes_2_int(vec));
	ros::Duration(1).sleep();
}


void DataCollector::close_UART()
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

int DataCollector::open_I2C()
{
	int file_i2c = 0;
	if((file_i2c = open(i2c_port,O_RDWR)) < 0)
	{
		cout << "open error\n";
		return -1;
	}

	int addr = 0x28;
	if(ioctl(file_i2c,I2C_SLAVE,addr) < 0)
	{
		cout << "ioctl error\n";
		return -1;
	}
	return file_i2c;
}


void DataCollector::print_to_I2C(int fd, uint8_t buffer[])
{
	ssize_t size = sizeof(buffer);

	if(write(fd,buffer,size) != size)
	{
		cout << "failed to write to i2c bus\n";
		return;
	}
}

uint8_t DataCollector::read_from_I2C(int fd)
{
	uint8_t buffer[] = {0x0};
	ssize_t size = sizeof(buffer);

	if(read(fd,buffer,size) != size)
	{
		cout << "failed to read from i2c bus\n";
		return 0x0;
	}
	else
	{
		//for(int i=0;i<size;i++)
		//	printf("%x ",buffer[i]);
		//cout << endl;
		return buffer[0];
	}
}

void DataCollector::close_I2C(int fd)
{
	close(fd);
}

void DataCollector::flush_serial_receiver()
{
	serial.flushReceiver();
}

void DataCollector::read_data_I2C(int bus, const string opt)
{
	uint8_t arr[2];
	uint8_t arr1[2];
	uint8_t arr2[2];
	uint8_t arr0[2]; // used only for quaternion: w

	if(opt == "ACC")
	{
		uint8_t read_X_MSB_command[] = {0x9};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x8};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0xB};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0xA};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0xD};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0xC};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("ACC: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));

	}else if(opt == "MAG")
	{
		uint8_t read_X_MSB_command[] = {0xF};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0xE};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x11};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x10};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x13};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x12};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("MAG: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));

	}else if(opt == "GYR")
	{
		uint8_t read_X_MSB_command[] = {0x15};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x14};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x17};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x16};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x19};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x18};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("GYR: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));
		
	}else if(opt == "EUL")
	{
		uint8_t read_X_MSB_command[] = {0x1B};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x1A};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x1D};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x1C};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x1F};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x1E};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("EUL: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));

		
	}else if(opt == "QUA")
	{
		uint8_t read_W_MSB_command[] = {0x21};
		print_to_I2C(bus,read_W_MSB_command);
		arr0[0] = read_from_I2C(bus);

		uint8_t read_W_LSB_command[] = {0x20};
		print_to_I2C(bus,read_W_LSB_command);
		arr0[1] = read_from_I2C(bus);

		uint8_t read_X_MSB_command[] = {0x23};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x22};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x25};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x24};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x27};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x26};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("QUA: (%d, %d, %d, %d)\n",convert_bytes_2_int(arr0),convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));
		
	}else if(opt == "LIA")
	{
		uint8_t read_X_MSB_command[] = {0x29};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x28};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x2B};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x2A};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x2D};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x2C};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("LIA: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));

		
	}else if(opt == "GRV")
	{
		uint8_t read_X_MSB_command[] = {0x2F};
		print_to_I2C(bus,read_X_MSB_command);
		arr[0] = read_from_I2C(bus);

		uint8_t read_X_LSB_command[] = {0x2E};
		print_to_I2C(bus,read_X_LSB_command);
		arr[1] = read_from_I2C(bus);

		uint8_t read_Y_MSB_command[] = {0x31};
		print_to_I2C(bus,read_Y_MSB_command);
		arr1[0] = read_from_I2C(bus);

		uint8_t read_Y_LSB_command[] = {0x30};
		print_to_I2C(bus,read_Y_LSB_command);
		arr1[1] = read_from_I2C(bus);

		uint8_t read_Z_MSB_command[] = {0x33};
		print_to_I2C(bus,read_Z_MSB_command);
		arr2[0] = read_from_I2C(bus);

		uint8_t read_Z_LSB_command[] = {0x32};
		print_to_I2C(bus,read_Z_LSB_command);
		arr2[1] = read_from_I2C(bus);

		printf("GRV: (%d, %d, %d)\n",convert_bytes_2_int(arr),convert_bytes_2_int(arr1),convert_bytes_2_int(arr2));
		
	}else if(opt == "TEMP")
	{
		uint8_t read_temp_command[] = {0x34};
		print_to_I2C(bus,read_temp_command);
		uint8_t tmp = read_from_I2C(bus);
		printf("TEMP: %d\n\n", tmp);
	}
}
*/

int main(int argc, char** argv)
{
	ros::init(argc,argv,"data_collector_node");

	ros::NodeHandle nh;

	ros::Rate rate(10);

	DataCollector dc = DataCollector(&nh);

	/*const uint8_t OPR_MODE[5] = {'\xAA','\x00','\x3D','\x01','\x08'}; // select operation mode: AMG
	const uint8_t read_ACC_DATA_X_MSB[4] = { '\xAA', '\x01', '\x09', '\x01'};
	const uint8_t read_ACC_DATA_X_LSB[4] = { '\xAA', '\x01', '\x08', '\x01'};
	const uint8_t read_CHIP_ID[4] = {'\xAA','\x01','\x00','\x01'};

	cout << sizeof(read_CHIP_ID) << endl;
	for(int i=0;i<sizeof(read_CHIP_ID);i++)
		printf("%x ",read_CHIP_ID[i]);
	cout << endl;
	*/

	int option = 0;
	cout << "Choose communication: (UART)0, (I2C)1, (SPI)2\n";
	cin >> option;

	switch(option)
	{
		case 0:
		{
			/*dc.open_UART();
			ros::Duration(1).sleep();

			dc.print_to_UART(read_CHIP_ID,4);
			ros::Duration(0.1).sleep();
			dc.read_from_UART();
			ros::Duration(1).sleep();
			
			dc.print_to_UART(OPR_MODE,5);
			ros::Duration(1).sleep();
			dc.read_from_UART();
			ros::Duration(1).sleep();
			
			while(ros::ok())
			{
				dc.read_data_UART("ACC");
				dc.read_data_UART("MAG");
				dc.read_data_UART("GYR");
				dc.read_data_UART("EUL");
				dc.read_data_UART("QUA");
				dc.read_data_UART("LIA");
				dc.read_data_UART("GRV");
			}
				
			dc.close_UART();
			*/

			UartCommunication uart = UartCommunication("/dev/ttyTHS1",115200);
			dc.communication = &uart;

			dc.communication->start();
			ros::Duration(1).sleep();

			while(ros::ok())
			{
				dc.communication->read_all_data("ACC");
				dc.communication->read_all_data("MAG");
				dc.communication->read_all_data("GYR");
				dc.communication->read_all_data("EUL");
				dc.communication->read_all_data("QUA");
				dc.communication->read_all_data("LIA");
				dc.communication->read_all_data("GRV");
				ros::Duration(1).sleep();
			}

			dc.communication->stop();
			
		}
		break;

		case 1:
		{
			// i2c code
			/*
			int bus = dc.open_I2C();
			
			uint8_t read_CHIP_ID_command[] = {0x0};
			dc.print_to_I2C(bus,read_CHIP_ID_command);
			dc.read_from_I2C(bus);

			while(ros::ok())
			{
				dc.read_data_I2C(bus,"ACC");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"MAG");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"GYR");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"EUL");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"QUA");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"LIA");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"GRV");
				ros::Duration(0.1).sleep();
				dc.read_data_I2C(bus,"TEMP");
				ros::Duration(0.1).sleep();
			}

			dc.close_I2C(bus);
			*/

			I2cCommunication i2c = I2cCommunication("/dev/i2c-2",0x57);
			dc.communication = &i2c;

			dc.communication->start();
			ros::Duration(1).sleep();

			while(ros::ok())
			{
				dc.communication->read_all_data("ACC");
				dc.communication->read_all_data("MAG");
				dc.communication->read_all_data("GYR");
				dc.communication->read_all_data("EUL");
				dc.communication->read_all_data("QUA");
				dc.communication->read_all_data("LIA");
				dc.communication->read_all_data("GRV");
				ros::Duration(1).sleep();
			}

			dc.communication->stop();

		}
		break;

		default:
		cout << "Please choose an option from [0,1] interval";
	}

	ros::spin();

	return 0;
}