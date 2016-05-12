%module "mumoro::DRegLC"

%{

#include "AlgoDRegLC.h"

%}

%include "std_vector.i"
namespace std{
	%template(vectornode) vector<Node>;
}

// Parse the original header file
%include "AlgoDRegLC.h"
