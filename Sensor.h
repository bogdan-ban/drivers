#include <iostream>
#include <string>

class Sensor
{
protected:
	std::string comunication_channel = "";

public:
	Sensor();
	Sensor(std::string new_comunication_channel)
	{
		comunication_channel = new_comunication_channel;
	}
	virtual void change_path(std::string new_comunication_channel) = 0;
};
