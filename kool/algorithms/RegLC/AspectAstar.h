/** Copyright : Ulrich Matchi Aïvodji (2015)  umaivodj@laas.fr

This software is a computer program whose purpose is to [describe
functionalities and technical features of your software].

This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use, 
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info". 

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability. 

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or 
data to be ensured and,  more generally, to use and operate it in the 
same conditions as regards security. 

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms. 
*/

#ifndef ASPECT_ASTAR_H
#define ASPECT_ASTAR_H

#include "DRegLC.h"
//#include "../utils/GeoTools.h"





namespace RLC {
    
struct AspectAstarParams {
    AspectAstarParams( const int target ) : target(target) {}
    const int target;
};


template<typename Base = DRegLC>
class AspectAstar : public Base 
{
public:    
    typedef LISTPARAM<AspectAstarParams, typename Base::ParamType> ParamType;
    
    std::set<RLC::Vertice> dest_vertices;
    int target_cost = -1;

    int az;
    
    AspectAstar( ParamType parameters ) : Base(parameters.next) {
        AspectAstarParams & p = parameters.value;
        
        BOOST_FOREACH( const int v_dfa, Base::graph->dfa_accepting_states() ) {
            dest_vertices.insert(Vertice(p.target, v_dfa));
        }

        az=p.target;
    }
    virtual ~AspectAstar() {}



    virtual Label label(RLC::Vertice vert, int time, int cost, int source = -1) const override {

        Label l = Base::label(vert, time, cost, source);

        const Transport::Graph * g=Base::graph->transport;

        Node start=g->mapNode(vert.first);

        Node end=g->mapNode(az);

        float lon1=start.lon, lat1=start.lat, lon2=end.lon, lat2=end.lat;


        double DEG_TO_RAD = 0.017453292519943295769236907684886;  
        // Earth's quatratic mean radius for WGS-84  
        double EARTH_RADIUS_IN_METERS = 6378137.0;  

        //Computes the arc, in radian, between two WGS-84 positions. 
        double latitudeArc  = (lat1 - lat2) * DEG_TO_RAD;  
        double longitudeArc = (lon1 - lon2) * DEG_TO_RAD; 

        double difflat=latitudeArc;
        double difflon=longitudeArc;


        double diffy=difflat*EARTH_RADIUS_IN_METERS;

        double diffx=difflon*EARTH_RADIUS_IN_METERS*cos(lat1*DEG_TO_RAD);



        //double latitudeH = sin(latitudeArc * 0.5); 
        //latitudeH *= latitudeH;  
        //double lontitudeH = sin(longitudeArc * 0.5);  
        //lontitudeH *= lontitudeH; 

        //double tmp = cos(lat1*DEG_TO_RAD) * cos(lat2*DEG_TO_RAD);  


       // double q= 2.0 * asin(sqrt(latitudeH + tmp*lontitudeH)); 
        // Computes the distance, in meters, between two WGS-84 positions. 
        //double dd=EARTH_RADIUS_IN_METERS*q;  

        //double ddd=sqrt( (lon1-lon2)*(lon1-lon2) + (lat1-lat2)*(lat1-lat2));

        double ddd=sqrt(diffx*diffx + diffy*diffy);
        
        l.h= (int) (ddd/10);
        
        //l.h=0;
        
        BOOST_ASSERT( l.valid() );
        return l;
    }

    
    
    virtual bool check_termination( const RLC::Label & lab ) override { 
        if( !(dest_vertices.find(lab.node) != dest_vertices.end()) ) {
            return false;
        } else {
            target_cost = lab.cost;
            return true;
        }
    }
    
    virtual bool finished() const override {
        return Base::finished() || Base::success;
    }

    
    
    
    std::list<int> get_path() const {
        std::list<int> path;
        RLC::Vertice curr;
        
        BOOST_FOREACH( RLC::Vertice v, dest_vertices ) {
            if(Base::black(v))
                curr = v;
        }
        
        path.push_front(curr.first);
        while( Base::has_pred(curr) ) {
            curr = Base::g->source(Base::get_pred( curr ));
            path.push_front(curr.first);
        }
        
        return path;
    }
    
    int get_path_cost() const {
//         BOOST_FOREACH( RLC::Vertice v, dest_vertices ) {
//             if(Base::black(v))
//                 return Base::cost( v );
//         }
        return target_cost;
    }
};


}


#endif
