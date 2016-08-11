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
// #include "ABBRobot.h"
#include "add.h"
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

//Define Global Variables
const int num_ineq_constraints = NUM_INEQ_CONSTRAINTS;
const int num_eq_constraints = NUM_EQ_CONSTRAINTS;
const int num_variables = NUM_VARIABLES;
const int num_constraints = num_ineq_constraints+num_eq_constraints;
std::vector<geometry_msgs::WrenchStamped> ft_wrenches;
pthread_mutex_t nonBlockMutex;

// Define structure to send arguments to thread
typedef struct thread_data{
    MatrixXd  *_q_pusher;
    MatrixXd  *_q_slider;
    MatrixXd  *_dq_slider;
    MatrixXd  *_Target;
    MatrixXd  *_u_control;
    MatrixXd  *_ap;
    double  *_tang_vel ;

} tdata_t;

struct thread_data thread_data_array[1];

//////////////////////////////////////////////////////////////////////////////
/////////////////////// QP Solver /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void *rriMain(void *thread_arg)
{
    struct thread_data *my_data;
    my_data = (struct thread_data *) thread_arg;

    //Define variables from argument pointers
    pthread_mutex_lock(&nonBlockMutex);
    MatrixXd *pq_slider = my_data->_q_slider;
    MatrixXd *pdq_slider= my_data->_dq_slider;
    MatrixXd *pq_pusher = my_data->_q_pusher;
    MatrixXd *pTarget = my_data->_Target;
    MatrixXd *pu_control = my_data->_u_control;
    MatrixXd *pap = my_data->_ap;
    double *ptang_vel = my_data->_tang_vel ;

    MatrixXd &q_slider = *pq_slider;
    MatrixXd &dq_slider = *pdq_slider;
    MatrixXd &q_pusher = *pq_pusher;
    MatrixXd &Target = *pTarget;
    MatrixXd &u_control = *pu_control;
    MatrixXd &ap = *pap;
    double &tang_vel  = *ptang_vel ;
    pthread_mutex_unlock(&nonBlockMutex);

    //Define local variables
    double fval1;
    double fval2;
    double fval3;
    double t_init;
    double T_des, delta_t, t_ini;
    double step_size = 1.f/100;
    int minIndex, maxCol;
    float min;

    //Define local matrix variables
    MatrixXd fval(3,1);
    MatrixXd _q_slider_(3,1);
    MatrixXd _dq_slider_(3,1);
    MatrixXd _q_pusher_(2,1);
    MatrixXd _Target_(2,1);

    //Define text file string variables
    char Q_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/H.txt";
    char Abar_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/A_bar.txt";
    char Ain_stick_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/Ain_stick.txt";
    char bin_stick_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/bin_stick.txt";
    char Aeq_stick_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/Aeq_stick.txt";
    char beq_stick_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/beq_stick.txt";
    char Ain_up_str[]    = "/home/mcube/cpush/catkin_ws/src/push_control/data/Ain_up.txt";
    char bin_up_str[]    = "/home/mcube/cpush/catkin_ws/src/push_control/data/bin_up.txt";
    char Aeq_up_str[]    = "/home/mcube/cpush/catkin_ws/src/push_control/data/Aeq_up.txt";
    char beq_up_str[]   = "/home/mcube/cpush/catkin_ws/src/push_control/data/beq_up.txt";
    char Ain_down_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/Ain_down.txt";
    char bin_down_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/bin_down.txt";
    char Aeq_down_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/Aeq_down.txt";
    char beq_down_str[] = "/home/mcube/cpush/catkin_ws/src/push_control/data/beq_down.txt";


    //Define object for 3 family of modes
    Push * pStick;
    Push * pUp;
    Push * pDown;

    pStick = new Push (Q_str, Abar_str, Ain_stick_str, bin_stick_str, Aeq_stick_str, beq_stick_str);
    pUp    = new Push (Q_str, Abar_str, Ain_up_str, bin_up_str, Aeq_up_str, beq_up_str);
    pDown  = new Push (Q_str, Abar_str, Ain_down_str, bin_down_str, Aeq_down_str, beq_down_str);

    Push &Stick= *pStick;
    Push &Up   = *pUp;
    Push &Down = *pDown;

    //Build optimization models
    Stick.build_model();
    Up.build_model();
    Down.build_model();

    //**********************************************************************
    //************************ Begin Loop **********************************
    //**********************************************************************
    double time = 0;
    double counter = 0;

    // cout<<  "Thread Loop Start"<< endl;

    while(time<50000 && ros::ok())
        {
        if (time==0){t_ini = gettime();}
        time = gettime()- t_ini;      

        //Read state of robot and object from shared variables
        pthread_mutex_lock(&nonBlockMutex);       
        _q_slider_ = q_slider;
        _dq_slider_= dq_slider;
        _q_pusher_= q_pusher;
        _Target_= Target;
        pthread_mutex_unlock(&nonBlockMutex);

        //Update and solve optimization models
        Stick.update_model(_q_slider_, _dq_slider_, _q_pusher_,_Target_);
        Up.update_model(_q_slider_, _dq_slider_, _q_pusher_,_Target_);
        Down.update_model(_q_slider_, _dq_slider_, _q_pusher_,_Target_);

        fval1 = Stick.optimize();
        fval2 = Up.optimize();
        fval3 = Down.optimize();

        //Find optimial control input
        fval << fval1, fval2, fval3;
        min = fval.minCoeff(&minIndex, &maxCol); //Find

        //Assign new control input to shared variables
        pthread_mutex_lock(&nonBlockMutex);
        if (minIndex==0){ u_control = Stick.delta_u; }
        else if (minIndex==1){  u_control = Up.delta_u;}
        else{   u_control = Down.delta_u;}
        // Determine desired velocity of pusher
        // MatrixXd Output(4,1);
        // Output = inverse_dynamics2(_q_pusher_, _q_slider_, _dq_slider_, u_control, tang_vel );
        // ap(0) = Output(0);
        // ap(1) = Output(1);
        // tang_vel   = Output(3);
        
        pthread_mutex_unlock(&nonBlockMutex);
        counter++;

        }
    //*********** End Loop **************************************************
    pthread_exit((void*) 0);
}


// helper function
uint32_t GetTickCount(void) 
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}


//////////////////////////////////////////////////////////////////////////
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const geometry_msgs::WrenchStamped& msg_force)
{
    geometry_msgs::WrenchStamped contact_wrench;
    contact_wrench.wrench = msg_force.wrench;
    ft_wrenches.push_back(contact_wrench);
        
}

// Create a simple robot message
void CreateSensorMessage(EgmSensor* pSensorMessage, float x, float y)
{ 
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

    EgmCartesian *pc = new EgmCartesian();
    //float z = 0.221;  //pu
    float z = 0.230;  //plywood
    if(x > 0.5) x = 0.5;
    if(x < 0.15) x = 0.15;
    if(y > 0.2) y = 0.2;
    if(y < -0.2) y = -0.2;
    pc->set_x(x*1000);    // convert to robot representation mm
    pc->set_y(y*1000);          
    pc->set_z(z*1000);
    EgmQuaternion *pq = new EgmQuaternion();
    pq->set_u0(0);   // need to fill in 
    pq->set_u1(0);
    pq->set_u2(1);
    pq->set_u3(0);

    EgmPose *pcartesian = new EgmPose();
    pcartesian->set_allocated_orient(pq);
    pcartesian->set_allocated_pos(pc);

    EgmPlanned *planned = new EgmPlanned();
    planned->set_allocated_cartesian(pcartesian);

    pSensorMessage->set_allocated_planned(planned);
}

void CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
{ 
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

}


void DisplayRobotMessage(EgmRobot *pRobotMessage, double& x, double& y, double& z)
{
    double x_robot, y_robot, z_robot;
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()  )
    {
        //printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
    x_robot =  pRobotMessage->feedback().cartesian().pos().x();
    y_robot =  pRobotMessage->feedback().cartesian().pos().y();
    z_robot =  pRobotMessage->feedback().cartesian().pos().z();
    
    x = x_robot / 1000;
    y = y_robot / 1000;
    z = z_robot /1000; 
    }
    else
    {
        printf("No header\n");
    }
}

bool getRobotPose(UDPSocket* EGMsock, string& sourceAddress, unsigned short& sourcePort, EgmRobot* pRobotMessage, double& robot_x, double& robot_y, double& robot_z){
    int recvMsgSize;
    const int MAX_BUFFER = 1400;
    char buffer[MAX_BUFFER];
    try{
        recvMsgSize = EGMsock->recvFrom(buffer, MAX_BUFFER-1, sourceAddress, sourcePort);
        if (recvMsgSize < 0)
        {
            printf("Error receive message\n");
        }
        else {
            //printf("Received %d\n", recvMsgSize);
        }

        // deserialize inbound message
        pRobotMessage->ParseFromArray(buffer, recvMsgSize);
        DisplayRobotMessage(pRobotMessage, robot_x, robot_y, robot_z); //Assign tcp position of robot to robot_x, robot_y, robot_z
        return true;
    } catch (SocketException &e) {}
    
    return false;
}

bool getViconPose(MatrixXd& q_slider, TransformListener& listener){
    tf::StampedTransform obj_pose;
    try{
        listener.lookupTransform("map", "vicon/StainlessSteel/StainlessSteel", ros::Time(0), obj_pose);
        tf::Quaternion q = obj_pose.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        q_slider << obj_pose.getOrigin().getX(), obj_pose.getOrigin().getY()+ 0.018, yaw;
        return true;
    }
    catch (tf::TransformException ex){
       //ROS_ERROR("%s",ex.what());
    }
    return false;
}

bool getViconVel(MatrixXd& dq_slider, TransformListener& listener){
    geometry_msgs::Twist obj_twist;
    try{
        listener.lookupTwist("vicon/StainlessSteel/StainlessSteel", "map", ros::Time(0), ros::Duration(0.5), obj_twist);
        dq_slider << obj_twist.linear.x, obj_twist.linear.y, obj_twist.angular.z;
        return true;
    }
    catch (tf::TransformException ex){
        //ROS_ERROR("%s",ex.what());
    }
    return false;
}

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
    //ros::Subscriber sub = n.subscribe("/tf", 1, tfCallback);


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

    //****************************************************************
    //************** Create Thread ****************************
    //****************************************************************
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

// !has_robot || !has_vicon_pos || 
    while((tmp < 1) && ros::ok())
    {
        tmp++;
        //Read robot position
        cout << " In first loop" << endl;
        cout << " tmp " << tmp << endl;
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

    
    for(int i=0;i<1;i++){
        cout << " In Second loop " << i << endl;
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, x_tcp, y_tcp, z_tcp)){
            has_robot = true;
            CreateSensorMessageEmpty(pSensorMessage);
            pSensorMessage->SerializeToString(&messageBuffer);
            EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        }
        usleep(4e3);
    }
    
    cout << " Second loop terminated" << endl;
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
    // ros::Rate r(100); // 10 hz

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
        
        // cout<< " Subscriber "<<endl;
        // ros::Subscriber sub = n.subscribe("netft_data", 1000, chatterCallback);
        ros::spinOnce();
        // cout << ft_wrenches.back()<< endl;
        
        geometry_msgs::WrenchStamped contact_wrench;
        contact_wrench =  ft_wrenches.back();
        cout << contact_wrench.wrench.force.x << endl;
        // cout << contact_wrench.wrench.force.y << endl;
        // cout << contact_wrench.wrench.force.z << endl;
        
        // add2(1,2);
        add(2,3);
        
        
        // Controller 1
        vp(0) = 0.00;
        vp(1) = 0.0;
        x_tcp = x_tcp + h*vp(0);
        y_tcp = y_tcp + h*vp(1);
        // printf(" %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n", q_slider(0), ros::spin();q_slider(1), q_slider(2), dq_slider(0), dq_slider(1), dq_slider(2), _x_tcp, _y_tcp, x_tcp, y_tcp, vp(0), vp(1), ap(0), ap(1));

          }

        // Send robot commands
        // CreateSensorMessage(pSensorMessage, x_tcp, y_tcp);
        // pSensorMessage->SerializeToString(&messageBuffer);
        // 
        // 
        // EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        
        //Sleep for 1000Hz loop
        usleep(1000);
        // r.sleep();
    }
     //async_spinner->stop();
    (void) pthread_join(rriThread, NULL);

    return 0;
}