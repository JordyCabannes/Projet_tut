%module "mumoro::ridesharing"

%{
	#include "header.h"
 	#include "aux_data.h"
 	#include "driver.h"
 	#include "rider.h"
 	#include "manager.h"
 	#include "toolbox.h"
 %}

// Parse the original header file

//%include "header.h"
%include "aux_data.h"
%include "driver.h"
%include "rider.h"
%include "manager.h"
%include "toolbox.h"

%template(IntVector) vector<int>;
%include "std_map.i"

namespace std {
    %template(map_int_node) map<int, Node>;
}


