from lib.core import mumoro
from lib.core.mumoro import Bike, Car, Foot, PublicTransport, GraphFactory, Driver, Pedestrian, Manager

import pymongo
from pymongo import MongoClient

client = MongoClient()
db=client.test_database
collection = db.test_collection

gf=GraphFactory("/home/matchi/Desktop/ST/graph.txt-dump", False)
g=gf.get()

n=g.num_edges()

for i in range(0, n):
  
  edge=g.map(i)
  node_s=g.source(edge.index)
  node_t=g.target(edge.index)
  s=g.mapNode(node_s)
  t=g.mapNode(node_t)
  
  hall = {"id": i, "road_edge": edge.road_edge, "edgemode": edge.type,"duration":g.get_duration(edge) , "source_id":node_s,"source_lon": s.lon, "source_lat": s.lat,"target_id":node_t, "target_lon": t.lon, "target_lat": t.lat}
  halls = db.halls
  hall_id = halls.insert(hall)
  
  print i
  
  
  
  