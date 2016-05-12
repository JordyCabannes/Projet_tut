#include "Carpooling/carpooling.h"



int main() {

    

    /*Transport::GraphFactory gf("/vagrant/dev/krog/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * g = gf.get();

    int car1_1=44636, car1_2=14767, foot1_1=35030, foot1_2=32509,
       car2_1=63187,car2_2=12690, foot2_1=39063, foot2_2=68048,
       car3_1=22068, car3_2=16910, foot3_1=15095, foot3_2=43768,
       car4_1=60631, car4_2=2576, foot4_1=25434, foot4_2=31328;

    Privacy::Driver * d1=new Privacy::Driver(car1_1, car1_2);
    Privacy::Driver * d2=new Privacy::Driver(car2_1, car2_2);
    Privacy::Driver * d3=new Privacy::Driver(car3_1, car3_2);
    Privacy::Driver * d4=new Privacy::Driver(car4_1, car4_2);

    Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot1_1, foot1_2);
    Privacy::Pedestrian * p2=new Privacy::Pedestrian(foot2_1, foot2_2);
    Privacy::Pedestrian * p3=new Privacy::Pedestrian(foot3_1, foot3_2);
    Privacy::Pedestrian * p4=new Privacy::Pedestrian(foot4_1, foot4_2);

    std::vector<Privacy::Driver> drivers;
    std::vector<Privacy::Pedestrian> passengers;

    drivers.push_back(d1);
    drivers.push_back(d2);
    drivers.push_back(d3);
    drivers.push_back(d4);

    passengers.push_back(p1);
    passengers.push_back(p2);
    passengers.push_back(p3);
    passengers.push_back(p4);*/


    carpooling::vertex_descriptor s,t;
    carpooling::Graph g;



    carpooling::SampleGraph::carpoolGraph(3,4,g, s, t);

    boost::edmonds_karp_max_flow(g, s, t);
    boost::cycle_canceling(g);

    int cost = boost::find_flow_cost(g);
    std::cout<<"cost: "<< cost <<std::endl;

   	boost::graph_traits<carpooling::Graph>::vertex_iterator vi, vend;

   	for(boost::tie(vi, vend) = vertices(g); vi != vend; ++vi)
    	std::cout << " node : " << *vi << std::endl;

    carpooling::Capacity capacity = get(boost::edge_capacity, g);
    carpooling::Weight weight = get(boost::edge_weight, g);
    carpooling::ResidualCapacity residual_capacity = get(boost::edge_residual_capacity, g);

  	boost::graph_traits <  carpooling::Graph  >::vertex_iterator u_iter, u_end;
  	boost::graph_traits <  carpooling::Graph  >::out_edge_iterator ei, e_end;

  	for (boost::tie(u_iter, u_end) = vertices(g); u_iter != u_end; ++u_iter)
    	for (boost::tie(ei, e_end) = out_edges(*u_iter, g); ei != e_end; ++ei)
      		if ( (capacity[*ei]-residual_capacity[*ei] ) > 0 && weight[*ei] > 0 )
        		std::cout << "node " << *u_iter << " with node " << target(*ei, g) << std::endl;

    
      
       

    return 0;
}

