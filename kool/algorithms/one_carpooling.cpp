/** Copyright : Ulrich Matchi AÏVODJI (2014)  umaivodj@laas.fr

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

#include "MultipleParticipants/MPR_AspectPrivacy.h"

int main(int argc, char*argv[]){

   Privacy::Toolbox * tb=new Privacy::Toolbox();



  /*if (argc != 2){

    cout << "Bad Input" << endl;
    exit(0);
  }
  else{
    string data=argv[1];
    tb->run_test(data);
  }



  tb->test_cd();

  tb->test_cc();

  tb->test_cc_with_limit();*/
  Transport::GraphFactory gf("/vagrant/dev/krog/graph.txt-dump", false);
  gf.setAll2();
  const Transport::Graph * g = gf.get();


  int car_1, car_2, foot_1, foot_2;


  car_1=30308;
  foot_1=26894;
  car_2=1175;
  foot_2=5949;

  Privacy::Driver * d1=new Privacy::Driver(car_1, car_2);
  Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot_1, foot_2);
  Privacy::Manager * m=new Privacy::Manager();

  Privacy::cd_output cd;
  Privacy::cc_output cc;

  cout << car_1 <<";"<< foot_1 <<";"<< car_2 <<";"<< foot_2 <<endl;

try
{
    cc=tb->cc_carpooling_test(*d1,*p1,g);
    if (cc.cc_costs.total_cost>0){
  cout << "======================================================================> solution"<<endl;
  delete d1;
  delete p1;

  }else{
  cout << "======================================================================> pas de solution"<<endl;
  delete d1;
  delete p1;
  }
}
catch ( const std::exception & e )
{
    //std::cerr << e.what();
    cout << "======================================================================> pas de solution"<<endl;
}




    delete g;

}





