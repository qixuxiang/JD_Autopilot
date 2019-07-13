#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
#include <sstream>
#include <angles/angles.h>
#include <global_coords.h>
using namespace std;
using namespace global_coords;


int main(int argc, char const *argv[]) {

  LatLonCoords latlon(39.7936580439358, 116.49891977436);  //initial latitude and longitude
  UtmCoords utm = latLonToUtm(latlon);          //convert to utm a first time
  cout << "utm.x: " << setprecision(18) <<utm.x << endl;   //x,double
  //setprecision for the decimal precision,and you need only set it once,all variable will follow it
  cout << "utm.y: " << utm.y << endl;           //y,double,it follow precision from x
  //cout << "utm.zone: " << utm.zone<< endl;     //zone,a string(char array in C)
  LatLonCoords ll = utmToLatLon(utm);           //convert back to lat lon
  cout << "ll.lat: " << ll.lat << endl;         //lat,double,it follow precision from x
  cout << "ll.lon: " << ll.lon << endl;         //lon,double,it follow precision from x

  return 0;
}
