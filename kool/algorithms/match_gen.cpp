#include "Carpooling/carpooling.h"

int compt_=0;

int main() {

    

    Transport::GraphFactory gf("/vagrant/dev/yooo/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * gr = gf.get();

    string filename="/vagrant/dev/yooo/analysis/output/projection/instances.csv";
    string path="/vagrant/dev/yooo/analysis/output/projection/matching.csv";
    carpooling::SampleGraph::save_to_carpooling(filename,gr,path);


    return 0;
}

