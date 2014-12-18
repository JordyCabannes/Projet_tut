#from lib.core import mumoro
#from lib.core.mumoro import Bike, Car, Foot, PublicTransport, GraphFactory, Driver, Pedestrian, Manager
from lib.core.mumoro import *
from toolbox import *


out=Output()

begin=time()   
gf= GraphFactory("/home/matchi/Desktop/ST/hercule.txt-dump", False)
end=time()
out.loading_time=end-begin

begin=time() 
g=gf.get() 
end=time()
out.geting_graph_time=end-begin

t=Toolbox()

tcf=t.test_loader("/home/matchi/Desktop/ST/config.csv")

print out.loading_time, out.geting_graph_time

print t.rgenerator(15,20)

t.test_cd()
  
  