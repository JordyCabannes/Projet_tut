#include "math.h"
// The usual PI/180 constant  
static const double DEG_TO_RAD = 0.017453292519943295769236907684886;  
// Earth's quatratic mean radius for WGS-84  
static const double EARTH_RADIUS_IN_METERS = 6372797.560856;  

//Computes the arc, in radian, between two WGS-84 positions. 

double ArcInRadians(float lon1, float lat1, float lon2, float lat2) {
  
  double latitudeArc  = (lat1 - lat2) * DEG_TO_RAD;  
  double longitudeArc = (lon1 - lon2) * DEG_TO_RAD;  
  double latitudeH = sin(latitudeArc * 0.5);  
  latitudeH *= latitudeH;  
  double lontitudeH = sin(longitudeArc * 0.5);  
  lontitudeH *= lontitudeH;  
  double tmp = cos(lat1*DEG_TO_RAD) * cos(lat2*DEG_TO_RAD);  
  return 2.0 * asin(sqrt(latitudeH + tmp*lontitudeH)); 

}  

// Computes the distance, in meters, between two WGS-84 positions. 

double haversine_dist(float lon1, float lat1, float lon2, float lat2) {  
  return EARTH_RADIUS_IN_METERS*ArcInRadians(lon1, lat1, lon2, lat2);  
} 

bool proximity(float lon1, float lat1, float lon2, float lat2,double radius){
  
  //cout<<"la distance est: "<<haversine_dist(lon1,lat1,lon2,lat2)<<endl;
  
  return (haversine_dist(lon1,lat1,lon2,lat2)<=radius);
}

double proximity_d(float lon1, float lat1, float lon2, float lat2){
  
  
  return haversine_dist(lon1,lat1,lon2,lat2);
}

bool verboseproximity(float lon1, float lat1, float lon2, float lat2,double radius){
  
  //cout<<"la distance est: "<<haversine_dist(lon1,lat1,lon2,lat2)<<" ";
  
  return (haversine_dist(lon1,lat1,lon2,lat2)<=radius);
}

bool proximityR(float lon1, float lat1, float lon2, float lat2,double radius){
  //cout<<"la distance est: "<<haversine_dist(lon1,lat1,lon2,lat2)<<endl;
  return (haversine_dist(lon1,lat1,lon2,lat2)>=radius);
}