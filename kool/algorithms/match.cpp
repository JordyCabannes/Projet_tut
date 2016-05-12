#include "Carpooling/carpooling.h"

int compt_=0;

int main() {

    Transport::GraphFactory gf("/vagrant/ubimob/graph.txt-dump", false);
    gf.setAll2();
    const Transport::Graph * gr = gf.get();

    string path="/vagrant/ubimob/analysis/output/projection/matching.csv";
    
    Users users=carpooling::SampleGraph::load(path);

    std::cout << "------------------ After ------------"<<std::endl;
    std::cout << "length drivers: " <<  users.drivers.size() <<std::endl;
    std::cout << "length pedestrians: " <<  users.passengers.size() <<std::endl;
    
    carpooling::vertex_descriptor s_heuristik,t_heuristik;
    carpooling::Graph g_heuristik;

    carpooling::vertex_descriptor s_ideal,t_ideal;
    carpooling::Graph g_ideal;

    //------------heuristic method

    std::cout<<"----------------- Geo distance heuristic ------------------"<<std::endl;

    carpooling::SampleGraph::heuristik_Graph(users,g_heuristik, s_heuristik, t_heuristik);
    boost::edmonds_karp_max_flow(g_heuristik, s_heuristik, t_heuristik);
    boost::cycle_canceling(g_heuristik);

    int cost_heuristik = boost::find_flow_cost(g_heuristik);
    std::cout<<"cost: "<< cost_heuristik <<std::endl;

    boost::graph_traits<carpooling::Graph>::vertex_iterator vi_heuristik, vend_heuristik;

    carpooling::Capacity capacity_heuristik = get(boost::edge_capacity, g_heuristik);
    carpooling::Weight weight_heuristik = get(boost::edge_weight, g_heuristik);
    carpooling::ResidualCapacity residual_capacity_heuristik = get(boost::edge_residual_capacity, g_heuristik);

    boost::graph_traits <  carpooling::Graph  >::vertex_iterator u_iter_heuristik, u_end_heuristik;
    boost::graph_traits <  carpooling::Graph  >::out_edge_iterator ei_heuristik, e_end_heuristik;

    int compt_heuristik=0;

    for (boost::tie(u_iter_heuristik, u_end_heuristik) = vertices(g_heuristik); u_iter_heuristik != u_end_heuristik; ++u_iter_heuristik){
        for (boost::tie(ei_heuristik, e_end_heuristik) = out_edges(*u_iter_heuristik, g_heuristik); ei_heuristik != e_end_heuristik; ++ei_heuristik){
            if ( (capacity_heuristik[*ei_heuristik]-residual_capacity_heuristik[*ei_heuristik] ) > 0 && weight_heuristik[*ei_heuristik] > 0 ){

                User _driver=carpooling::SampleGraph::findByPk_driver(*u_iter_heuristik-1,users);
                Privacy::Driver *d1=new Privacy::Driver(_driver.origin, _driver.destination);
                User _pedestrian= carpooling::SampleGraph::findByPk_passenger(target(*ei_heuristik, g_heuristik)-users.drivers.size()-1,users);
                Privacy::Pedestrian *p1=new Privacy::Pedestrian(_pedestrian.origin, _pedestrian.destination);
                int cost1;
                Privacy::cc_output cc;
                Privacy::Toolbox *tb=new Privacy::Toolbox();
                cc=tb->cc_carpooling_test(*d1,*p1,gr);
                cost1=cc.cc_costs.total_cost;

                std::cout << "node " << *u_iter_heuristik << " with node " << target(*ei_heuristik, g_heuristik) << " with cost: " << cost1 << std::endl;

                compt_heuristik ++;

                delete d1;
                delete p1;
                delete tb;
            }
        }
    }

    std::cout<<"----------------- Number of matching ------------------ : " << compt_heuristik <<std::endl;         

//-----------------ideal algorithm------------------

    std::cout<<"----------------- ideal algorithm ------------------"<<std::endl;
    

    carpooling::SampleGraph::ideal_Graph(users,g_ideal, s_ideal, t_ideal,gr);
    boost::edmonds_karp_max_flow(g_ideal, s_ideal, t_ideal);
    boost::cycle_canceling(g_ideal);

    int cost_ideal = boost::find_flow_cost(g_ideal);
    std::cout<<"cost: "<< cost_ideal <<std::endl;

    boost::graph_traits<carpooling::Graph>::vertex_iterator vi_ideal, vend_ideal;
    carpooling::Capacity capacity_ideal = get(boost::edge_capacity, g_ideal);
    carpooling::Weight weight_ideal = get(boost::edge_weight, g_ideal);
    carpooling::ResidualCapacity residual_capacity_ideal = get(boost::edge_residual_capacity, g_ideal);

    boost::graph_traits <  carpooling::Graph  >::vertex_iterator u_iter_ideal, u_end_ideal;
    boost::graph_traits <  carpooling::Graph  >::out_edge_iterator ei_ideal, e_end_ideal;

    int compt_ideal=0;

    for (boost::tie(u_iter_ideal, u_end_ideal) = vertices(g_ideal); u_iter_ideal != u_end_ideal; ++u_iter_ideal){
        for (boost::tie(ei_ideal, e_end_ideal) = out_edges(*u_iter_ideal, g_ideal); ei_ideal != e_end_ideal; ++ei_ideal){
            if ( (capacity_ideal[*ei_ideal]-residual_capacity_ideal[*ei_ideal] ) > 0 && weight_ideal[*ei_ideal] > 0 ){

                User _driver1=carpooling::SampleGraph::findByPk_driver(*u_iter_ideal-1,users);
                Privacy::Driver *d2=new Privacy::Driver(_driver1.origin, _driver1.destination);
                User _pedestrian1= carpooling::SampleGraph::findByPk_passenger(target(*ei_ideal, g_ideal)-users.drivers.size()-1,users);
                Privacy::Pedestrian *p2=new Privacy::Pedestrian(_pedestrian1.origin, _pedestrian1.destination);
                int cost2;
                Privacy::cc_output cc2;
                Privacy::Toolbox *tbb=new Privacy::Toolbox();
                cc2=tbb->cc_carpooling_test(*d2,*p2,gr);
                cost2=cc2.cc_costs.total_cost;
                std::cout << "node " << *u_iter_ideal << " with node " << target(*ei_ideal, g_ideal) << " with cost: " << cost2 << std::endl;

                compt_ideal ++;

                delete d2;
                delete p2;
                delete tbb;
            }
        }
    }
    std::cout<<"----------------- Number of matching ------------------ : " << compt_ideal <<std::endl; 





    return 0;
}

