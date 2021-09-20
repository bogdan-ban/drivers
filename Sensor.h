#include <iostream>
#include <string>

class Sensor
{
protected:
	std::string comunication_channel = "";

public:
	virtual void change_path(std::string new_comunication_channel) = 0;
};
