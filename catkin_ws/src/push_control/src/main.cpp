// Include header files
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
#include "pushing.h"
#include "helper.h"
#include "ABBRobot.h"
#include "OptProgram.h"
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

//Name shortcuts
using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;

//Define Global Variables
const int num_ineq_constraints = NUM_INEQ_CONSTRAINTS;
const int num_eq_constraints = NUM_EQ_CONSTRAINTS;
const int num_variables = NUM_VARIABLES;
const int num_constraints = num_ineq_constraints+num_eq_constraints;
std::vector<geometry_msgs::WrenchStamped> ft_wrenches;
pthread_mutex_t nonBlockMutex;
struct thread_data thread_data_array[1];

//********************************************************************
// Main Program
//********************************************************************
int
main(int argc,  char *argv[])
{
    ros::init(argc, argv, "push_control");
    ros::NodeHandle n;
    tf::TransformListener listener;
    ros::Subscriber sub = n.subscribe("/netft_data", 1, chatterCallback);
    
    // Declare Matrix variables
    MatrixXd q_pusher(2,1);
    MatrixXd dq_pusher(2,1);
    MatrixXd q_slider(3,1);
    MatrixXd dq_slider(3,1);
    MatrixXd r_cb_b(2,1);
    MatrixXd Cbi(3,3);
    MatrixXd Cbi_T(3,3);
    MatrixXd Target(2,1);
    MatrixXd u_control(4,1);
    MatrixXd vp(2,1);
    MatrixXd ap(2,1);
    MatrixXd _q_slider_(3,1);
    MatrixXd _dq_slider_(3,1);
    MatrixXd smoothed_dq_slider(3,1);
    MatrixXd _q_pusher_(2,1);
    MatrixXd _q_pusher_sensor(2,1);
    MatrixXd _Target_(2,1);
    MatrixXd _u_control_(4,1);
    MatrixXd vo_des(3,1);
    MatrixXd qo_des(3,1);

    //Define physical parameters
    const double a = 0.09;
    double psi, theta;
    double minIndex, maxCol;
    double dX, dY;
    double fval1, fval2, fval3;
    double t_init;
    double time;
    double T_des, delta_t, t_ini;
    double step_size = 1.f/250;
    double xp, yp, zp;
    double x_tcp, y_tcp, z_tcp;
    double _x_tcp, _y_tcp, _z_tcp;
    double xp_des, yp_des;
    double tcp_width = 0.00475;
    bool has_robot = false;
    bool has_vicon_pos = false;
    bool has_vicon_vel = false;
    double Position_Outputs[2];
    double tang_vel=0;
    FILE *myFile = NULL;

    //Define Mutex
    pthread_mutex_t nonBlockMutex;
    pthread_mutex_init(&nonBlockMutex, NULL);

    //Define Thread
    pthread_t rriThread;
    pthread_attr_t attrR;
    pthread_attr_init(&attrR);
    pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);

    //Assign arguments to pass to thread
    thread_data_array[0]._q_pusher = &q_pusher;
    thread_data_array[0]._q_slider = &q_slider;
    thread_data_array[0]._dq_slider = &dq_slider;
    thread_data_array[0]._Target = &Target;
    thread_data_array[0]._u_control = &u_control;
    thread_data_array[0]._ap = &ap;
    thread_data_array[0]._tang_vel  = &tang_vel;
        
    // Create socket and wait for robot's connection
    UDPSocket* EGMsock;
    const int portNumber = 6510;
    string sourceAddress;             // Address of datagram source
    unsigned short sourcePort;        // Port of datagram source

    EGMsock = new UDPSocket(portNumber);
    EgmSensor *pSensorMessage = new EgmSensor();
    EgmRobot *pRobotMessage = new EgmRobot();
    string messageBuffer;
    
    //Initialize Vicon and robot data collection
    int tmp = 0;
    int lv1 = 0;
    double xs_old=0, xs=0,dxs=0;
    double ys_old=0, ys=0,dys=0 ;
    double thetas_old=0, thetas=0, dthetas=0;
    double h1=1.0f/1000;

    //First Loop
    while(!has_robot || !has_vicon_pos || (tmp < 2500) && ros::ok())
    {
        tmp++;
        //Read robot position
        // cout << " In first loop" << endl;
        // cout << " tmp " << tmp << endl;
        // cout << " has_vicon_pos "<< has_vicon_pos << endl;
        // cout << " has_robot "    << has_robot << endl;
        // cout << " ros::ok() " << ros::ok() << endl;
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, x_tcp, y_tcp, z_tcp)){
                        has_robot = true;
            //CreateSensorMessageEmpty(pSensorMessage);
            CreateSensorMessage(pSensorMessage,0.23,-0.04);
            pSensorMessage->SerializeToString(&messageBuffer);
            EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        }
        if(getViconPose(q_slider, listener))
            has_vicon_pos = true;
        xs_old = xs;
        ys_old = ys;
        thetas_old = thetas; 
        
        xs = smooth(q_slider(0), 0.98, xs);
        ys = smooth(q_slider(1), 0.98, ys);
        thetas = smooth(q_slider(2), 0.98, thetas);
        
        dq_slider(0) = (xs-xs_old)/h1;
        dq_slider(1) = (ys-ys_old)/h1;
        dq_slider(2) = (thetas-thetas_old)/h1;

        lv1+=1;
        usleep(4e3);
    }

    // return 0;
    //Read Vicon and Initialize Variables
    theta = q_slider(2);

    Target << 2.5,-0.040;
    q_pusher(0) = x_tcp;
    q_pusher(1) = y_tcp;

    //Create Thread
    pthread_create(&rriThread, &attrR, rriMain, (void*) &thread_data_array[0]) ;

    //Second Loop
    for(int i=0;i<1000;i++){
        // cout << " In Second loop " << i << endl;
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, x_tcp, y_tcp, z_tcp)){
            has_robot = true;
            CreateSensorMessageEmpty(pSensorMessage);
            pSensorMessage->SerializeToString(&messageBuffer);
            EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        }
        usleep(4e3);
    }
    // cout << " Second loop terminated" << endl;
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, _x_tcp, _y_tcp, _z_tcp)){
            _q_pusher_sensor<<_x_tcp,_y_tcp;
        }
    

    //****************************************************************
    //************** Main Control Loop ****************************
    //****************************************************************
    double psi_des = 0;
    double dx_smooth=0;
    double dy_smooth=0;
    vp << 0,0;
    double _x_tcp_old = 0.0;
    //std::cout << "async spinner" << std::endl;
    //async_spinner = new ros::AsyncSpinner(2);
    //async_spinner->start();
    ros::Rate r(1000); // 10 hz

    for (int i=0;i<15000  && ros::ok();i++){

        if (i==0){t_ini = gettime();}
        time = gettime()- t_ini;
        // cout << i << endl;

        //**********************  Get State of robot and object from ROS *********************************
       // if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, _x_tcp, _y_tcp, _z_tcp)){
           // _q_pusher_sensor<<_x_tcp,_y_tcp;
        // }         
        
        pthread_mutex_lock(&nonBlockMutex);
        getViconPose(q_slider, listener);
        xs_old = xs;
        ys_old = ys;
        thetas_old = thetas; 
        
        xs = smooth(q_slider(0), 0.98, xs);
        ys = smooth(q_slider(1), 0.98, ys);
        thetas = smooth(q_slider(2), 0.98, thetas);
        
        dq_slider(0) = (xs-xs_old)/h1;
        dq_slider(1) = (ys-ys_old)/h1;
        dq_slider(2) = (thetas-thetas_old)/h1;

        q_pusher(0) = x_tcp;// + tcp_width*cos(theta*1);
        q_pusher(1) = y_tcp;// + tcp_width*sin(theta*1);
        
        //Assign local variables
        _q_slider_ = q_slider;
        _dq_slider_ = dq_slider;
        _q_pusher_ = q_pusher;
        _Target_ = Target;
        _u_control_ = u_control;
        pthread_mutex_unlock(&nonBlockMutex);
        //**********************************************************************************
        double h = 1.0f/1000;
        //wait for 1 sec before starting position control 
        if (time<=1)
          {x_tcp = x_tcp;
                        // cout << " In first condition "  << endl;
          // cout << " time "  <<time<< endl;
          }

        else if (time>=1 and time <=1.3)
        {    vp(0) = 0.05;
             vp(1) = 0;
        x_tcp = x_tcp + h*vp(0);
        y_tcp = y_tcp + h*vp(1);
        }
        else
        {
        MatrixXd Output(4,1);
        Output = inverse_dynamics2(_q_pusher_, _q_slider_, _dq_slider_, u_control, tang_vel, time);
        ap(0) = Output(0);
        ap(1) = Output(1);
        tang_vel   = Output(3);
        
        // Controller 1
        // vp(0) = vp(0) + 1*h*ap(0);
        // vp(1) = vp(1) + 1*h*ap(1);
        // x_tcp = x_tcp + h*vp(0) + .5*h*h*ap(0);
        // y_tcp = y_tcp + h*vp(1) + .5*h*h*ap(1);
        
        ros::spinOnce();
        geometry_msgs::WrenchStamped contact_wrench;
        contact_wrench =  ft_wrenches.back();
        // cout << contact_wrench.wrench.force.x << endl;
        
        
        // Controller 2
        vp(0) = 0.05;
        vp(1) = 0.0;
        x_tcp = x_tcp + h*vp(0);
        y_tcp = y_tcp + h*vp(1);
        printf(" %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n", q_slider(0), q_slider(1), q_slider(2), dq_slider(0), dq_slider(1), dq_slider(2), _x_tcp, _y_tcp, x_tcp, y_tcp, vp(0), vp(1), ap(0), ap(1),contact_wrench.wrench.force.x,contact_wrench.wrench.force.y,contact_wrench.wrench.force.z);

          }

        // Send robot commands
        CreateSensorMessage(pSensorMessage, x_tcp, y_tcp);
        pSensorMessage->SerializeToString(&messageBuffer);
        EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        
        //Sleep for 1000Hz loop
        // usleep(1000);
        r.sleep();
    }
     //async_spinner->stop();
    (void) pthread_join(rriThread, NULL);

    return 0;
}
