/** Copyright : Arthur Bit-Monnot (2013)  arthur.bit-monnot@laas.fr

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

#ifndef ASPECT_TARGET_AREA_STOP_H
#define ASPECT_TARGET_AREA_STOP_H

#include "../utils/Area.h"
#include"../../lib/core/utils.h"
#include"DRegLC.h"


namespace RLC {

struct AspectTargetAreaStopParams {
    AspectTargetAreaStopParams( Area * area ) : area(area) {}
    Area * area;
};
    
/**
 * This Aspect add a stop condition when all car accessible nodes in the area have been set
 */
template<typename Base>
class AspectTargetAreaStop : public Base
{
private:
    int car_nodes_set = 0;
    Area * area;
    
public:
    
    typedef LISTPARAM<AspectTargetAreaStopParams, typename Base::ParamType> ParamType;
    
    AspectTargetAreaStop(ParamType p) : Base( p.next )
    {
        area = p.value.area;
    }

    virtual Label treat_next() override {
        RLC::Label l = Base::treat_next();
        if( area->isIn( l.node.first ) && Base::graph->transport->car_accessible( l.node.first ) ) {
            ++car_nodes_set;  
	    cout<<"yeaaah"<<endl;
        }
        return l;
    }
    
    virtual bool finished() const override {
        return Base::finished() || (car_nodes_set >= area->num_car_accessible);
    }
};



}


#endif