#ifndef ADD_BEHAVIOR_INCLUDED
#define ADD_BEHAVIOR_INCLUDED

void lane_keeping(vector<vector<double>>  sensor_car_list_current, double car_s, double prev_s, double s_buffer,
   double &v_init, double &v_end, double car_speed, bool &flag){

  if ( abs(car_speed-v_end) < 0.1 && flag == false) {
       cout << setw(25) << "Target speed reached: " << car_speed << endl;
       flag = true;
       car_v_init_global = car_speed;
       car_v_end_global = car_speed;
       v_init = car_v_init_global;
       v_end = car_v_end_global;
  }

  for(int i=0; i<sensor_car_list_current.size(); i++){

    if(sensor_car_list_current[sensor_car_list_current.size()-i-1][1]>car_s){

      if (sensor_car_list_current[sensor_car_list_current.size()-i-1][1]<prev_s+ s_buffer){

        double car_v_target = sensor_car_list_current[sensor_car_list_current.size()-i-1][3];
        //cout << car_v_target << " " << ((car_v_target-v_end)< 0.5*lane_keeping_buffer_v) << " " << (car_v_target < car_speed_max) << " " << flag << endl;

        if (flag == true) {
          if (car_v_target > car_speed_max && v_end < car_speed_max){
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Increase speed to: " << car_speed_max  << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_speed_max;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if( (car_v_target < car_speed_max) && ((car_v_target-v_end)< 0.5*lane_keeping_buffer_v) ) {
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Decrease speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if( car_v_target<car_speed_max && car_v_target-v_end > 1.5*lane_keeping_buffer_v){
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Increase speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
        }

      }

    }
  }
}

#endif // ADD_BEHAVIOR_INCLUDED
