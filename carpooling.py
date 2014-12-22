#from lib.core import mumoro
#from lib.core.mumoro import Bike, Car, Foot, PublicTransport, GraphFactory, Driver, Pedestrian, Manager
from lib.core.mumoro import *
from toolbox import *
from modules.psi_2_round_elgamal import PSI2RoundElGamal

def timer(func, *pargs, **kargs):
    """
    Measures the time required to run func with the given parameters.
    Returns the time as well as the result of the computation.
    """
    start = time()
    ret = func(*pargs, **kargs)
    elapsed = time() - start
    return elapsed, ret


def run_psi_2_round_elgamal(server_set, client_set):
    """
    Simulates the 2-round PSI protocol based on ElGamal encryption scheme
    on the given server and client sets. Returns the final output of the client.
    """
    psi = PSI2RoundElGamal()
    client_out_1, client_state = psi.client_to_server(client_set)
    server_out = psi.server_to_client(server_set, **client_out_1)
    client_out_2 = psi.client_output(server_out, **client_state)
    return client_out_2



output=Output()

gf= GraphFactory("graph.txt-dump", False)
gf.setAll2()
g=gf.get() 

t=Toolbox()




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

#print psi_pickup

#print psi_dropoff 

pickup_driver=d1.getPickup()
pickup_pedestrian=p1.getPickup()

dropoff_driver=d1.getDropoff()
dropoff_pedestrian=p1.getDropoff()


print "driver pickup",len(pickup_driver)
print "pedestrian pickup",len(pickup_pedestrian)

print "driver dropoff",len(dropoff_driver)
print "pedestrian dropoff",len(dropoff_pedestrian)

pickup_driver_set=[]
pickup_pedestrian_set=[]

dropoff_driver_set=[]
dropoff_pedestrian_set=[]

for val in pickup_driver:
  pickup_driver_set.append(val)

for val in pickup_pedestrian:
  pickup_pedestrian_set.append(val)


for val in dropoff_driver:
  dropoff_driver_set.append(val)

for val in dropoff_pedestrian:
  dropoff_pedestrian_set.append(val)


inter_pickup=set(pickup_pedestrian_set) & set(pickup_driver_set)
print "inter pickup", len(inter_pickup)

inter_dropoff=set(dropoff_pedestrian_set) & set(dropoff_driver_set)
print "inter dropoff", len(inter_dropoff)

time_taken_pickup, result_pickup = timer(run_psi_2_round_elgamal, pickup_driver_set, pickup_pedestrian_set)

print time_taken_pickup, len(result_pickup)

time_taken_dropoff, result_dropoff = timer(run_psi_2_round_elgamal, dropoff_driver_set, dropoff_pedestrian_set)

print time_taken_dropoff, len(result_dropoff)

#psi_result=m.getPSI(psi_pickup, psi_dropoff)

psi_result=m.getPSI(result_pickup, result_dropoff)

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
