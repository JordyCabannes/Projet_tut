
#include <stdlib.h>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
#include <time.h>
#include "GraphFactory.h"


int main() 
{
    Transport::GraphFactory gf(2);

    
    gf.add_road_edge(0, 1, FootEdge, 35);
    
     
     gf.set_pickUp(0);

    
    gf.save_to_txt("/home/matchi/Desktop/ST/zulu.txt-dump");

   
   

    
}
