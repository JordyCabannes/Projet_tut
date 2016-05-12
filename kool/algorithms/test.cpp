#include "Model/Model.h"

int main(int argc, char const *argv[])
{
	
	std::map<int, Driver> drivers;

	for (int i = 0; i < 10; ++i)
	{
		Driver d(i);
		d.model=Hot;
		drivers.insert(std::pair<int,Driver>(i, d));
	}

	std::cout<<"number of drivers: "<<drivers.size()<<std::endl;

	std::cout<<"4th driver model: "<< Driver::findByPk(3,drivers).model <<std::endl;
	

	return 0;
}