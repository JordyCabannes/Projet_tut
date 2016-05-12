#pragma once
#include <iostream>
#include <string>
#include <stdlib.h>
#include <map>
#include "math.h"
#include "Model.h"
namespace pbm {

struct Driver
{

private:
	Journey trip;
	int id;
	std::vector<RiderVector> receivedVectors;


public:
	Driver(int id, Journey trip);
	void receiveVector(RiderVector rV);
	void computeProjection(int on);



};

}