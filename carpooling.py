#from lib.core import mumoro
#from lib.core.mumoro import Bike, Car, Foot, PublicTransport, GraphFactory, Driver, Pedestrian, Manager
from lib.core.mumoro import *
from toolbox import *

output=Output()

gf= GraphFactory("/home/hercule.txt-dump", False)
gf.setAll2()
g=gf.get() 

t=Toolbox()

car1=13089
foot1=13384
car2=38925
foot2=71781


d1=Driver(car1, car2)
p1=Pedestrian(foot1, foot2)

m=Manager()

limit_car=600 +1
limit_foot=300 +1

      
p1.findPickup(g,limit_foot)

p1.findDropoff(g,limit_foot)

d1.findPickup(g,limit_car)    

d1.findDropoff(g,limit_car)      
  
psi_pickup=m.PSI_Pickup(d1, p1)
    
psi_dropoff=m.PSI_Dropoff(d1, p1)	    

psi_result=m.getPSI(psi_pickup, psi_dropoff)

m.GetAllPath(g, psi_result)



if(m.shared_path_len()>0):
  
  d1.getFavorites(m.shared_path)
    
  p1.getFavorites(m.shared_path)
  
  m.match(d1,p1);
  
  
  output.cd_pickup=m.getThePath().start
  output.cd_dropoff=m.getThePath().end
  
	
  output.cd_costs=t.cd_costs(d1,p1,m,g)
  
  
  len1=output.cd_costs.len_foot_pickup
  len2=output.cd_costs.len_car_pickup
  len3=output.cd_costs.len_shared_path
  len4=output.cd_costs.len_foot_dropoff
  len5=output.cd_costs.len_car_dropoff
  wait_time=abs(len1-len2)
  total_cost=len1+len2+2*len3+len4+len5+ wait_time
  
  print "len foot1: ", len1
  print "len car1: ", len2
  print "len shared: ", len3
  print "len foot2: ", len4
  print "len car2: ", len5
  print "total cost: ",total_cost
  
  print "_____________postions__________"

  print "foot1: ", foot1
  print "car1: ", car1
  
  print "pickup: ", output.cd_pickup
  print "dropoff: ", output.cd_dropoff
  
  print "foot2: ", foot2
  print "car2: ", car2
else:
  print "============================> pas de solution"
