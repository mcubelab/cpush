/*
 */
#ifndef ABBRobot_H
#define ABBRobot_H

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <ctime>
#include <math.h>
#include <pthread.h>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <fstream>
#include <memory>
#include <cstdlib>
// #include "pushing.h"
// #include "ABBRobot.h"
#include <time.h>
#include <iomanip>
#include <sys/time.h>
#include <sys/resource.h>

#include "PracticalSocket/PracticalSocket.h" // For UDPSocket and SocketException
#include "egm.pb.h" // generated by Google protoc.exe
#include "tf2_msgs/TFMessage.h"
#include "tf/LinearMath/Transform.h"

#include <ros/ros.h>
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"

using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;
using Eigen::ArrayXf;

void add2(double a, double b);



#endif
