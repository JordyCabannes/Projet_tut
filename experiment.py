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

def run_test(filename):

	output=Test_output()
	gf= GraphFactory("graph.txt-dump", False)
	gf.setAll2()
	g=gf.get() 
	tb=Toolbox()
	tcf=tb.test_loader(filename)
	tb.create_result_file("output.csv")
	k=0

	while (k<tcf.number_of_scenarios):
	  
	  print "################### Scenario ", k, "################################"
	  synchro_1=tb.synchro(g)
	  synchro_2=tb.get_satellite(tb.satellites(synchro_1,tcf.synchro2_a,tcf.synchro2_b,g))

	  car_1=tb.get_satellite(tb.satellites_free(synchro_1, tcf.car1_a,tcf.car1_b,g))
	  foot_1=tb.get_satellite(tb.satellites_free(synchro_1, tcf.foot1_a,tcf.foot1_b,g))
	 
	  car_2=tb.get_satellite(tb.satellites_free(synchro_2, tcf.car2_a,tcf.car2_b,g))
	  foot_2=tb.get_satellite(tb.satellites_free(synchro_2, tcf.foot2_a,tcf.foot2_b,g))
	      
	  d1=Driver(car_1, car_2)
	  p1=Pedestrian(foot_1, foot_2)
	  m=Manager()

	  limit_car=tcf.limit_car +1
	  limit_foot=tcf.limit_foot +1

	  begin=time()        
	  p1.findPickup(g,limit_foot)
	  end=time()
	  output.iso_foot_pickup_time=str(end-begin)

	  begin=time()
	  p1.findDropoff(g,limit_foot)
	  end=time()
	  output.iso_foot_dropoff_time=str(end-begin)


	  begin=time()
	  d1.findPickup(g,limit_car)
	  end=time()
	  output.iso_car_pickup_time=str(end-begin)

	  begin=time()
	  d1.findDropoff(g,limit_car)
	  end=time()
	  output.iso_car_dropoff_time=str(end-begin)

	  pickup_driver_set=d1.getPickup()
	  pickup_pedestrian_set=p1.getPickup()

	  dropoff_driver_set=d1.getDropoff()
	  dropoff_pedestrian_set=p1.getDropoff()


	  # print "driver pickup",len(pickup_driver)
	  # print "pedestrian pickup",len(pickup_pedestrian)

	  # print "driver dropoff",len(dropoff_driver)
	  # print "pedestrian dropoff",len(dropoff_pedestrian)

	  psi_pickup=m.PSI_Pickup(d1, p1)    
	  psi_dropoff=m.PSI_Dropoff(d1, p1)	

	  #inter_pickup=set(pickup_pedestrian_set) & set(pickup_driver_set)
	  # print "inter pickup", len(inter_pickup)

	  #inter_dropoff=set(dropoff_pedestrian_set) & set(dropoff_driver_set)
	  # print "inter dropoff", len(inter_dropoff)

	  #psi_result=m.getPSI(result_pickup, result_dropoff)

	  psi_result=m.getPSI(psi_pickup, psi_dropoff)

	  begin=time()
	  m.GetAllPath(g, psi_result)
	  end=time()
	  output.path_computing_time=str(end-begin)

	  if(m.shared_path_len()>0):
	        
	    time_taken_pickup, result_pickup = timer(run_psi_2_round_elgamal, pickup_driver_set, pickup_pedestrian_set)
	    print time_taken_pickup, len(result_pickup)
	    output.psi_pickup_time=str(time_taken_pickup)
	    output.psi_pickup_size=len(result_pickup)

	    time_taken_dropoff, result_dropoff = timer(run_psi_2_round_elgamal, dropoff_driver_set, dropoff_pedestrian_set)
	    print time_taken_dropoff, len(result_dropoff)
	    output.psi_dropoff_time=str(time_taken_dropoff)
	    output.psi_dropoff_size=len(result_dropoff)


	    begin=time()
	    d1.getFavorites(m.shared_path)
	    end=time()
	    output.path_ordering_car_time=str(end-begin)

	    begin=time()
	    p1.getFavorites(m.shared_path)
	    end=time()
	    output.path_ordering_foot_time=str(end-begin)

	    begin=time()
	    m.match(d1,p1)
	    end=time()
	    output.path_election_time=str(end-begin)
	      
	    output.scenario_id=k
	    
	    #id of positions
	    output.car_start=d1.posStart
	    output.car_end=d1.posEnd
	    output.foot_start=p1.posStart
	    output.foot_end=p1.posEnd
	    #id of pickup/dropoff
	    output.cd_pickup=m.getThePath().start
	    output.cd_dropoff=m.getThePath().end
	    #postions of the distributed solution
	    output.cd_positions=tb.cd_carpooling_positions(d1,p1,m,g);
	    #costs of the distributed solution 
	    output.cd_costs=tb.cd_costs(d1,p1,m,g);
	    #size of potential pickup for pedesdrian
	    output.pickup_foot_size=tb.dlen(p1.data_before)
	    #size of potential pickup for driver
	    output.pickup_car_size=tb.dlen(d1.data_before)
	    #size of potential dropoff for pedesdrian
	    output.dropoff_foot_size=tb.dlen(p1.data_after)
	    #size of potential dropoff for driver
	    output.dropoff_car_size=tb.dlen(d1.data_after)
	    #centralized tests
	    cc=tb.cc_carpooling_test(d1,p1,g);
	    ccl=tb.cc_carpooling_test_with_limit(d1,p1,g,limit_foot+1,limit_car+1);
	    #costs in centralized
	    output.cc_costs=cc.cc_costs;
	    #pickup in centralized
	    output.cc_pickup=cc.cc_pickup;
	    #dropoff in centralized
	    output.cc_dropoff=cc.cc_dropoff;
	    #position in centralized
	    output.cc_positions=cc.cc_positions;
	    #runtime in centralized
	    output.cc_time=cc.cc_time;
	    #costs in centralized
	    output.ccl_costs=ccl.cc_costs;
	    #pickup in centralized
	    output.ccl_pickup=ccl.cc_pickup;
	    #dropoff in centralized
	    output.ccl_dropoff=ccl.cc_dropoff;
	    #position in centralized
	    output.ccl_positions=ccl.cc_positions;
	    #runtime in centralized
	    output.ccl_time=ccl.cc_time;
	    output.shared_path_size=m.shared_path_len()
	    tb.save_all("output.csv", output, g);

	  else:

	    print "===============> pas de solution"

	  k=k+1

run_test("config.csv")
