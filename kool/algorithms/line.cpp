#include "Carpooling/carpooling.h"

int compt_=0;

int main() {

    

    Transport::GraphFactory gf("/vagrant/dev/yooo/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * gr = gf.get();

    
    cout << my_map.size() << endl;


    return 0;
}

