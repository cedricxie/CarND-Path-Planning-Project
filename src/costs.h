#ifndef ADD_COST_INCLUDED
#define ADD_COST_INCLUDED

double comfort = 0.1;
double collision = 1.0;
double efficiency = 0.6;

double cost_lane_change(double d_init, double d_end){
  if( abs(d_init - d_end)< 0.01*car_lane_width ){
    return 0.0;
  }
  else{
    return comfort;
  }
}

double cost_speed(double car_speed){
  return efficiency*(1.0-car_speed/car_speed_max);
}

double cost_collision(double car_s, double car_speed, double d_end, vector<vector<double>>  sensor_car_list_end){

  int idx = -1;

  for(int i=0; i<sensor_car_list_end.size(); i++){
    if(sensor_car_list_end[sensor_car_list_end.size()-i-1][1]>car_s){ idx = i;}
  }

  if (idx == -1){
    double s1 = sensor_car_list_end[0][1];
    double v1 = sensor_car_list_end[0][3];
    if( s1+v1*2.0*t_inc+lane_change_buffer > car_s+car_speed*2.0*t_inc){return collision;}
  }
  else if(idx==sensor_car_list_end.size()-1){
    double s2 = sensor_car_list_end[idx][1];
    double v2 = sensor_car_list_end[idx][3];
    if( s2+v2*2.0*t_inc < car_s+car_speed*2.0*t_inc+lane_change_buffer ){return collision;}
  }
  else{
    double s1 = sensor_car_list_end[idx-1][1];
    double v1 = sensor_car_list_end[idx-1][3];
    if( s1+v1*2.0*t_inc+lane_change_buffer > car_s+car_speed*2.0*t_inc){return collision;}
    double s2 = sensor_car_list_end[idx][1];
    double v2 = sensor_car_list_end[idx][3];
    if( s2+v2*2.0*t_inc < car_s+car_speed*2.0*t_inc+lane_change_buffer ){return collision;}
  }
  return 0.0;
}

#endif // ADD_COST_INCLUDED
