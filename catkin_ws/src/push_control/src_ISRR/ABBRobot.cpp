#include  "HeaderFiles.h"

using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;

uint32_t GetTickCount(void) 
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

// *********************************
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
void chatterCallback(const geometry_msgs::WrenchStamped& msg_force)
{
    geometry_msgs::WrenchStamped contact_wrench;
    contact_wrench.wrench = msg_force.wrench;
    ft_wrenches.push_back(contact_wrench);
        
}
// *********************************
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
    if(x > 0.55) x = 0.55;
    if(x < 0.10) x = 0.10;
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
// ***********************************
void CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
{ 
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

}

// ************************
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

// ****************************
bool getRobotPose(UDPSocket* EGMsock, string& sourceAddress, unsigned short& sourcePort, EgmRobot* pRobotMessage, double& robot_x, double& robot_y, double& robot_z)
{
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

// *****************************
bool getViconPose(MatrixXd& q_slider, TransformListener& listener){
    tf::StampedTransform obj_pose;
    try{
        listener.lookupTransform("map", "vicon/StainlessSteel/StainlessSteel", ros::Time(0), obj_pose);
        tf::Quaternion q = obj_pose.getRotation();
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
         
        
        q_slider << obj_pose.getOrigin().getX()- 0.0, obj_pose.getOrigin().getY()+ 0.0, yaw - 0*0.205681;
        return true;
    }
    catch (tf::TransformException ex){
       //ROS_ERROR("%s",ex.what());
    }
    return false;
}


// ****************************8
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
