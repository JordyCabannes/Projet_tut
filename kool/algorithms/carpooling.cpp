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
  
   //Privacy::Toolbox * tb=new Privacy::Toolbox();

    
  
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

  typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
  boost::char_separator<char> sep(";");
  string filename="/vagrant/dev/krog/genData/goku.csv";
  ifstream in(filename.c_str());
  if (!in.is_open()) exit(0);

  vector< string > vec;
  string line;
  int i=0;

  int car_1, car_2, foot_1, foot_2;


  while (getline(in,line)){

      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());

      if(i>0){

      car_1=std::atoi(vec.at(0).c_str());
      foot_1=std::atoi(vec.at(1).c_str());
      car_2=std::atoi(vec.at(2).c_str());
      foot_2=std::atoi(vec.at(3).c_str());
      Privacy::Toolbox * tb=new Privacy::Toolbox();
      Privacy::Driver * d1=new Privacy::Driver(car_1, car_2);
      Privacy::Pedestrian * p1=new Privacy::Pedestrian(foot_1, foot_2);
      Privacy::cd_output cd;
      Privacy::cc_output cc, ccl;
      Privacy::Manager * m=new Privacy::Manager();
      cout << "ligne " << i <<endl;

      cout << car_1 <<";"<< foot_1 <<";"<< car_2 <<";"<< foot_2 <<endl;

      cd=tb->cd_carpooling_test(*d1,*p1,*m,g, 300, 60);

      if (cd.cd_costs.total_cost>0){
      cout << "======================================================================> solution"<<endl;
      delete tb;
      delete d1;
      delete p1;
      delete m;

      }else{
      cout << "======================================================================> pas de solution"<<endl;
      delete tb;
      delete d1;
      delete p1;
      delete m;
      }




      }
      i++;
    }

    delete g;



}
  



	
