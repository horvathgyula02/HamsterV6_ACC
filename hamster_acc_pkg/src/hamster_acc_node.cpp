// Header file-ok

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int32.h>
#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <numeric>
#include <hamster_acc_pkg/plot_info.h>

using namespace std;


// Globális változók

double Kp = 0.3;
double Ki = 0.00001;
double Kd = 0.01;
double dt = 0.5;

int key_input;
float set_speed;
float steering_angle;
float avg_dist;
bool acc_switch = false;
bool cc_switch = false;
float actual_speed;
vector<float> speed_container(5);
vector<float> turnrad_container(5);
float avg_dist_old;
float wheelbase = 0.165;
float avg_speed_diff;
float avg_turnrad;
float obj_trunrad;
float closest_turn_obj_range = 5;

double target = 1;
double target_diff = 0.1;
double target_lower = target - target_diff;
double target_upper = target + target_diff;
double inspeed;

double prev_error;



ackermann_msgs::AckermannDriveStamped key_control;
ros::Publisher ackermann_cmd_pub;
ros::Publisher plot_pub;
hamster_acc_pkg::plot_info plot_info;

// Billentyű lenyomásásnak beolvasásáért felelős függvény

int keypress_reading(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

// Lenyomott billentyű ASCII érték alapján történő érzékelés, jármű manuális billentyűkkel történő irányításásért feleleős függvény

void keyboard_control(void)
{
  if(keypress_reading()){
    key_input = getchar();
  }

  if(key_input == 105){                               // i
    Ki += 0.00001;                                    
  }
  else if(key_input == 107){                          // k
    Ki -= 0.00001;
  }

  if(key_input == 111){                               // o
    Kp += 0.01;
  }
  else if(key_input == 108){                          // l
    Kp -= 0.01;
  }

  if(key_input == 117){                               // u
    target += 0.1;
  }
  else if ( key_input == 106){                        // j
    target -= 0.1;
  }

  if(key_input == 119){                               // w
    set_speed += 0.05;

    if(set_speed >= 0.5){
      set_speed = 0.5;
    }
  }

  else if(key_input == 115){                          // s
    set_speed -= 0.05;

    if(set_speed <= -0.5){
      set_speed = -0.5;
    }
  }

  else if(key_input == 97){                           // a
    steering_angle += 0.01;

    if(steering_angle >= 0.5){
      steering_angle = 0.5;
    }
  }

  else if(key_input == 100){                           // d
    steering_angle -= 0.01;

    if(steering_angle <= -0.5){
      steering_angle = -0.5;
    }
  }

  else if(key_input == 9){                            // TAB
    steering_angle = 0;
  }

  else if(key_input == 32){                           // Space
    set_speed = 0;
  }

  else if(key_input == 116){                          // t
    cc_switch = !cc_switch;
  }

  else if(key_input == 122 & cc_switch == true){      // z
    acc_switch = !acc_switch;
  }

  if(cc_switch == false){
    acc_switch = false;
  }

  key_input=0;
}

// Lidar szenzor pontfelhőjének feldolgozása, szűrése, mérése

double calculatePIDOutput(double kp, double ki, double kd, double target, double current, double dt, double& prev_error, double& integral)
{
    double error = target - current;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return output;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  system("clear");
  
  // Lidar pontfelhő feldolgozás

  vector<float> actual_ranges;
  vector<vector<float>> x_y_ranges;
  vector<float> single_x_y_range(2);
  vector<float> zero(2);

  zero[0] = 0;
  zero[1] = 0;

  for (int i = 0; i < data->ranges.size(); i++){
    actual_ranges.push_back(data->ranges[i]);

  }

  for(int i = 0; i < actual_ranges.size(); i++){                                    // Robot előtt 180°-ban x és y koordináta kiszámolása

    /*if((i > 89) & (i < 180)){
      single_x_y_range[0] = (actual_ranges[i] * sin((180-i)*(M_PI/180)));           // Valós teszteset során használt x,y koordináta számítás
      single_x_y_range[1] = (actual_ranges[i] * cos((180-i)*(M_PI/180)));
    }

    else if((i >179) & (i < 270)){
      single_x_y_range[0] = actual_ranges[i] * cos((270-i)*(M_PI/180))*(-1);
      single_x_y_range[1] = actual_ranges[i] * sin((270-i)*(M_PI/180));
    }*/

    if(i < 90){
      single_x_y_range[0] = (actual_ranges[i] * sin(i*(M_PI/180)))*(-1);            // Gazebo szimuláció során használt x,y koordináta számítás
      single_x_y_range[1] = (actual_ranges[i] * cos(i*(M_PI/180)));
    }

    else if((i < 360) & (i > 269)){
      single_x_y_range[0] = actual_ranges[i] * sin((360-i)*(M_PI/180));
      single_x_y_range[1] = actual_ranges[i] * cos((360-i)*(M_PI/180));
    }

    if (single_x_y_range[0] >= (-0.2) & single_x_y_range[0] <= 0.2){
      x_y_ranges.push_back(single_x_y_range);
    }
    
    else{
      x_y_ranges.push_back(zero);
    }

  }

  // Legközelebbi objektum átlagos távolságának kiszámítása

  int counter = 0;
  float y_sum = 0;
  float prev_y_range = 0;
  float diff = 0.05;

  for (int i = 0; i < x_y_ranges.size(); i++){                                  // Trajektóriumban lévő tárgy/jármű egyes pontjainak y irányú távolsága
    float prev_y_lower_bound = prev_y_range - diff;
    float prev_y_upper_bound = prev_y_range + diff;


    if(x_y_ranges[i][1] > 0){
      if(x_y_ranges[i][1] >= prev_y_lower_bound & x_y_ranges[i][1] <= prev_y_upper_bound || prev_y_range == 0){
        y_sum += x_y_ranges[i][1];
        counter += 1;
        prev_y_range = x_y_ranges[i][1];
      }
      
      else if(x_y_ranges[i][1] < prev_y_lower_bound){
        y_sum = 0;
        counter = 0;
        y_sum += x_y_ranges[i][1];
        counter += 1;
        prev_y_range = x_y_ranges[i][1];
      }

      else if(x_y_ranges[i][1] > prev_y_upper_bound){
        y_sum = y_sum;
        counter = counter;
      }
    }
  }

  avg_dist = (y_sum/counter);                                                   // Trajektóriumban lévő tárgy/jármű átlagos
  
  counter = 0;
  cout << "Distance: " << avg_dist << '\n'; 
  plot_info.avg_dist = avg_dist;

  // Sebesség különbség számítása

  float actual_speed_diff = (avg_dist-avg_dist_old)/0.1;  

  avg_dist_old = avg_dist;

  speed_container[4] = speed_container[3];
  speed_container[3] = speed_container[2];
  speed_container[2] = speed_container[1];
  speed_container[1] = speed_container[0];
  speed_container[0] = actual_speed_diff;

  avg_speed_diff = (speed_container[0] + speed_container[1] + speed_container[2] + speed_container[3] + speed_container[4])/5;  
  plot_info.rel_vel = avg_speed_diff;

  // Fordulókör kiszámítása

  float steering_angle_deg = (key_control.drive.steering_angle * (180/M_PI));

  turnrad_container[4] = turnrad_container[3];
  turnrad_container[3] = turnrad_container[2];
  turnrad_container[2] = turnrad_container[1];
  turnrad_container[1] = turnrad_container[0];
  turnrad_container[0] = wheelbase/(tan(steering_angle));

  avg_turnrad = (turnrad_container[0] + turnrad_container[1] + turnrad_container[2] + turnrad_container[3] + turnrad_container[4])/5;

  // PID control

  static double integral = 0;
  double error;

  if(key_control.drive.steering_angle == 0){
    error = target - avg_dist;
  }
  else{
    error = target - closest_turn_obj_range;
  }

  double proportional = Kp * error;

  // Calculate integral term
  integral += error * dt;
  double integralTerm = Ki * integral;

  double derivative = ((error - prev_error)/dt) * Kd;

  // Calculate control signal

  double controlSignal = proportional + integralTerm + derivative;





  // Robot irányítása

  actual_speed = key_control.drive.speed;
  float control_speed = actual_speed;
  cout << "CC: " << cc_switch << "        (0 - off   1 - on)"<< '\n' << "ACC: " << acc_switch << '\n' << "Set speed: " << set_speed << " [m/s]" << '\n' << "Actual speed: " << actual_speed << " [m/s]" << '\n' << "Steering angle: " << steering_angle << " [rad]" << '\n';
  
  // Egyenes irányú mozgás esetén, nincs kormányszögelfordulás

  keyboard_control();
  //key_control.drive.steering_angle = steering_angle;
  ackermann_cmd_pub.publish(key_control);
  if(actual_speed == 0){
    key_control.drive.steering_angle = 0;
  }
  else{
    key_control.drive.steering_angle = steering_angle;
  }
  
  if(key_control.drive.steering_angle == 0){

    if(cc_switch == false){
      key_control.drive.speed = 0;
    }

    else if(cc_switch == true & acc_switch == false){                                                      // Robot sebességének állítása gombokkal, abban az esetben, ha ACC ki van kapcsolva
      key_control.drive.speed = set_speed;
    }

    else if(cc_switch == true & acc_switch == true){

      control_speed -= controlSignal;

      if(control_speed > set_speed){
        control_speed = set_speed;
      }
      if(control_speed < 0){
        control_speed = 0;
      }

      key_control.drive.speed = control_speed;

      if(avg_speed_diff > 0.05 || avg_speed_diff < -0.05 || avg_dist > target_upper || avg_dist < target_lower){
        key_control.drive.speed = control_speed;
      }
      
      else if(avg_speed_diff <= 0.05 || avg_speed_diff >= -0.05 || avg_dist <= target_upper || avg_dist > target_lower){
        key_control.drive.speed = actual_speed;
      }
    }
  }

  // Kanyarodás esetén

  else{

    for (int i = 0; i < actual_ranges.size(); i++){

      if(actual_ranges[i] != 0){

        if(i < 90){
          obj_trunrad = sin((i*M_PI/180)) * avg_turnrad;
        }
        if(i > 269){
          obj_trunrad = sin(((i-269)*M_PI/180)) * avg_turnrad;
        }

        if(obj_trunrad > (avg_turnrad - 0.1) & obj_trunrad < (avg_turnrad + 0.1)){
          float turn_obj_range;

          // balra kanyar
          if(key_control.drive.steering_angle > 0 & (i < 90 | i > 330)){
            turn_obj_range = actual_ranges[i];
          }
          if(key_control.drive.steering_angle < 0 & (i < 30 | i > 269)){
            turn_obj_range = actual_ranges[i];
          }

          if(turn_obj_range < closest_turn_obj_range & turn_obj_range > 0.01){
            closest_turn_obj_range = turn_obj_range;
          }
        }
      }
      else{
        closest_turn_obj_range = 5;
      }

    }

    if(cc_switch == false){
      key_control.drive.speed = 0;
    }

    else if(cc_switch == true & acc_switch == false){                                                      // Robot sebességének állítása gombokkal, abban az esetben, ha ACC ki van kapcsolva
      key_control.drive.speed = set_speed;
    }

    else if(cc_switch == true & acc_switch == true){

      control_speed -= controlSignal;

      if(control_speed > set_speed){
        control_speed = set_speed;
      }
      if(control_speed < 0){
        control_speed = 0;
      }

      key_control.drive.speed = control_speed;

      if(avg_speed_diff > 0.02 || avg_speed_diff < -0.02 || closest_turn_obj_range > target_upper || closest_turn_obj_range < target_lower){
        key_control.drive.speed = control_speed;
      }
      
      else if(avg_speed_diff <= 0.02 || avg_speed_diff >= -0.02 || closest_turn_obj_range <= target_upper || closest_turn_obj_range > target_lower){
        key_control.drive.speed = actual_speed;
      }
    }


  }
  if(controlSignal < 0){
    controlSignal = 0;
  }
  plot_info.control_signal = controlSignal;

  cout << "control signal: " << controlSignal << '\n' << "control speed: " << control_speed << '\n' << "Ki: " << Ki << '\n' << "Kp: " << Kp << '\n' << "target: " << target << '\n' << "error: " << error << '\n' << "Sebesség különbség: " << avg_speed_diff << '\n' << "inspeed: " << inspeed << '\n' << "kormányszög: " << steering_angle_deg << "°" << '\n' << "fordulókör sugár: " << avg_turnrad << '\n';
  cout << "obj turn range: " << closest_turn_obj_range << '\n';
  plot_pub.publish(plot_info);


}

int main(int argc, char** argv)
{
  
  // Initialize the ROS node

  ros::init(argc, argv, "hamster_acc_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Create a subscriber to the LIDAR topic
  ros::Subscriber sub_lidar = nh.subscribe("/agent1/scan", 100, lidarCallback);
  plot_pub = nh.advertise<hamster_acc_pkg::plot_info>("/plotinfo",1);
  ackermann_cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/agent1/ackermann_cmd", 1);

  // Spin the ROS node
  ros::spin();

  return 0;
}