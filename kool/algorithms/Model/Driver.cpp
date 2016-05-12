#pragma once
#include <iostream>
#include <string>
#include <stdlib.h>
#include <map>
#include "math.h"
#include "Model.h"

namespace pbm {

Driver::Driver(int the_id, Journey the_trip) : id(the_id), trip(the_trip){}

Driver::receiveVector(RiderVector rV){
	receiveVectors.push_back(rV);
}

Driver::computeProjection(int on){
	
}	


	
};

}