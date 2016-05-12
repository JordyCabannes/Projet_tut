%module "mumoro::carpooling"
%include boost.i 

%{

 #include "carpooling.h"

%}

// Parse the original header file

%include "carpooling.h"


//%ignore std::vector<User>::vector(size_type);
//%include <std_vector.i>
%template(vectorUser) std::vector<User>;


