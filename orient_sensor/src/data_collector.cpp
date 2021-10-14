#include "data_collector.h"


DataCollector::DataCollector(ros::NodeHandle *nh)
{
	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"data_collector_node");

	ros::NodeHandle nh;

	ros::Rate rate(10);

	DataCollector dc = DataCollector(&nh);	

	int option = 0;
	cout << "Choose communication: (UART)0, (I2C)1, (SPI)2\n";
	cin >> option;

	switch(option)
	{
		case 0:
		{
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