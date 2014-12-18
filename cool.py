from lib.core import mumoro
from lib.core.mumoro import Bike, Car, Foot, PublicTransport, GraphFactory, Driver, Pedestrian, Manager

import pymongo
from pymongo import MongoClient

client = MongoClient()
db=client.test_database
collection = db.test_collection

halls=db.halls
gf=GraphFactory(76285)

print "lorem ipsum"

for edge in halls.find():
  
  if (edge['road_edge']):
  
    print edge['id']
    gf.add_road_edge(edge['source_id'], edge['target_id'], edge['edgemode'], edge['duration'])
  
    gf.set_coord(edge['source_id'], edge['source_lon'], edge['source_lat'])
    gf.set_coord(edge['target_id'], edge['target_lon'], edge['target_lat'])
  
    gf.set_pickUp(edge['source_id'])
    gf.set_pickUp(edge['target_id'])
  
  
  if not (edge['road_edge']):
    
    #add_public_transport_edge(const int source,const  int target, const int duration, const EdgeMode type);
    
    gf.add_public_transport_edge(edge['source_id'], edge['target_id'],edge['duration'], edge['edgemode'])
    print "ola ola"
    gf.set_coord(edge['source_id'], edge['source_lon'], edge['source_lat'])
    gf.set_coord(edge['target_id'], edge['target_lon'], edge['target_lat'])
   
      
gf.save_to_txt("/home/matchi/Desktop/ST/zulu.txt-dump")
    
    
