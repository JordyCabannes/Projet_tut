#pragma once
#include "Model.h"

// The usual PI/180 constant  
static const double _DEG_TO_RAD = 0.017453292519943295769236907684886;  
// Earth's quatratic mean radius for WGS-84  
static const double _EARTH_RADIUS_IN_METERS = 6372797.560856;  

//Computes the arc, in radian, between two WGS-84 positions. 
double _ArcInRadians(Location L1, Location L2) {
  
  double latitudeArc  = (L1.lat - L2.lat) * _DEG_TO_RAD;  
  double longitudeArc = (L1.lon - L2.lon) * _DEG_TO_RAD;  
  double latitudeH = sin(latitudeArc * 0.5);  
  latitudeH *= latitudeH;  
  double lontitudeH = sin(longitudeArc * 0.5);  
  lontitudeH *= lontitudeH;  
  double tmp = cos(L1.lat*_DEG_TO_RAD) * cos(L2.lat*_DEG_TO_RAD);  
  return 2.0 * asin(sqrt(latitudeH + tmp*lontitudeH)); 

}  

// Computes the distance, in meters, between two WGS-84 positions. 

double Hdist(Location L1, Location L2) {  
  return _EARTH_RADIUS_IN_METERS*_ArcInRadians(L1, L2);  
} 

double TwoHdist(Journey J1, Journey J2){
  return Hdist(J1.origin, J2.origin)+Hdist(J1.destination,J2.destination)
}
