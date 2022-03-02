

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
// ZMQ includes for messaging
#include "zhelpers.hpp"
#include<zmq.hpp>
// Local includes
#include<time.h>
#include<chrono>
#include<string.h>
#include<iostream>
#include<array>

#include "pub_socket.h"


pub_socket pub_sock;
// geometry_msgs::PoseStamped base_pose; 

pub_socket::pub_socket() : context(1), publisher(context,ZMQ_PUB), sub_context(1), subscriber(sub_context,ZMQ_SUB), rot_q{0.7071,-0.7071,0.0,0.0} {
    publisher.setsockopt(ZMQ_CONFLATE, 1); 
    publisher.bind("tcp://127.0.0.1:3885");
}


// void Callback(const geometry_msgs::PoseStamped& ps)
// {
//     // Print out received data
//    //ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n \t w:[%f] \n",
//    //           ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
//    //           ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w); 

//     // PixHawk pose that is to be transformed from inertial frame of Optitrack (X-Forward,Y-Up,Z-Right) into PixHawk NED convention

//     geometry_msgs::PoseStamped top_pose; 
   
//     // Transformation of position into NED frame
//     top_pose.pose.position.x = ps.pose.position.x;
//     top_pose.pose.position.y = ps.pose.position.z;
//     top_pose.pose.position.z = -ps.pose.position.y;

//     top_pose.pose.orientation.w = ps.pose.orientation.w;
//     top_pose.pose.orientation.x = ps.pose.orientation.x;
//     top_pose.pose.orientation.y = ps.pose.orientation.z;
//     top_pose.pose.orientation.z = -ps.pose.orientation.y;

//     // Print out after mapping before sending
//     // std::cout<<"\n Sample count is : "<<std::endl;
//     // ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t w:[%f] \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n",
//     //           pix_pose.pose.position.x,pix_pose.pose.position.y,pix_pose.pose.position.z,
//     //           pix_pose.pose.orientation.w,pix_pose.pose.orientation.x,pix_pose.pose.orientation.y,pix_pose.pose.orientation.z);

//          //Declare a ZMQ message queue  
//             zmq::message_t message(70);
    
//     // Serialize (convert the data into a string) over zmq message AS PER THE ATT_POS_MOCAP_MSG PACK
//             snprintf((char *) message.data(),70, "%f %f %f %f %f %f %f", top_pose.pose.orientation.w,top_pose.pose.orientation.x,top_pose.pose.orientation.y,top_pose.pose.orientation.z,
//             	top_pose.pose.position.x,top_pose.pose.position.y,top_pose.pose.position.z);
    
//     // Publish the message 
//           pub_sock.publisher.send(message);
    
//     // Frequency of publishing should be between 30-50 Hz. Currently set to 50 Hz.
//           sleep(0.02); 
    
// }


// void Callback2(const geometry_msgs::PoseStamped& ps)
// {
//     // Print out received data
//    //ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n \t w:[%f] \n",
//    //           ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
//    //           ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w); 

//     // PixHawk pose that is to be transformed from inertial frame of Optitrack (X-Forward,Y-Up,Z-Right) into PixHawk NED convention

  
//    geometry_msgs::PoseStamped base_pose; 
//     // Transformation of position into NED frame
//     base_pose.pose.position.x = ps.pose.position.x;
//     base_pose.pose.position.y = ps.pose.position.z;
//     base_pose.pose.position.z = -ps.pose.position.y;

//     base_pose.pose.orientation.w = ps.pose.orientation.w;
//     base_pose.pose.orientation.x = ps.pose.orientation.x;
//     base_pose.pose.orientation.y = ps.pose.orientation.z;
//     base_pose.pose.orientation.z = -ps.pose.orientation.y;

//     // Print out after mapping before sending
//     std::cout<<"\n Sample count is : "<<std::endl;
//           sleep(0.02); 
    

// }

// void Callback3(const geometry_msgs::PoseStampedConstPtr& ps, const geometry_msgs::PoseStampedConstPtr& ps2)
// {
// // Print out received data
// //ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n \t w:[%f] \n",
// //           ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
// //           ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w); 

// // PixHawk pose that is to be transformed from inertial frame of Optitrack (X-Forward,Y-Up,Z-Right) into PixHawk NED convention
//     geometry_msgs::PoseStamped top_pose; 
//     geometry_msgs::PoseStamped base_pose; 

// // Transformation of position into NED frame
//     top_pose.pose.position.x = ps->pose.position.x;
//     top_pose.pose.position.y = ps->pose.position.z;
//     top_pose.pose.position.z = ps->pose.position.y;

//     top_pose.pose.orientation.w = ps->pose.orientation.w;
//     top_pose.pose.orientation.x = ps->pose.orientation.x;
//     top_pose.pose.orientation.y = ps->pose.orientation.z;
//     top_pose.pose.orientation.z = ps->pose.orientation.y;

//     base_pose.pose.position.x = ps2->pose.position.x;
//     base_pose.pose.position.y = ps2->pose.position.z;
//     base_pose.pose.position.z = ps2->pose.position.y;

//     base_pose.pose.orientation.w = ps2->pose.orientation.w;
//     base_pose.pose.orientation.x = ps2->pose.orientation.x;
//     base_pose.pose.orientation.y = ps2->pose.orientation.z;
//     base_pose.pose.orientation.z = ps2->pose.orientation.y;

// // Print out after mapping before sending
//     // std::cout<<"\n Sample count is : "<<std::endl;
//     // ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t w:[%f] \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n",
//     //           top_pose.pose.position.x,top_pose.pose.position.y,top_pose.pose.position.z,
//     //           top_pose.pose.orientation.w,top_pose.pose.orientation.x,top_pose.pose.orientation.y,top_pose.pose.orientation.z
//     // );

// //Declare a ZMQ message queue  
//     zmq::message_t message(140);
    
// // Serialize (convert the data into a string) over zmq message AS PER THE ATT_POS_MOCAP_MSG PACK
//     snprintf((char *) message.data(),140, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
//                 base_pose.pose.position.x,base_pose.pose.position.y,base_pose.pose.position.z,
//                 base_pose.pose.orientation.w,base_pose.pose.orientation.x,base_pose.pose.orientation.y,base_pose.pose.orientation.z,
//                 top_pose.pose.position.x,top_pose.pose.position.y,top_pose.pose.position.z,
//                 top_pose.pose.orientation.w,top_pose.pose.orientation.x,top_pose.pose.orientation.y,top_pose.pose.orientation.z);
    
// // Publish the message 0
//     pub_sock.publisher.send(message);
    
// // Frequency of publishing should be between 30-50 Hz. Currently set to 50 Hz.
//     sleep(0.02); 
// }

void Callback4(const geometry_msgs::PoseStampedConstPtr& ps0, const geometry_msgs::PoseStampedConstPtr& ps1)
{
// Print out received data
//ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n \t w:[%f] \n",
//           ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,
//           ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w); 

// PixHawk pose that is to be transformed from inertial frame of Optitrack (X-Forward,Y-Up,Z-Right) into PixHawk NED convention
    geometry_msgs::PoseStamped top_pose; 
    geometry_msgs::PoseStamped base_pose; 

// Transformation of position into NED frame
    base_pose.pose.position.x = ps0->pose.position.x;
    base_pose.pose.position.y = ps0->pose.position.z;
    base_pose.pose.position.z = ps0->pose.position.y;

    base_pose.pose.orientation.w = ps0->pose.orientation.w;
    base_pose.pose.orientation.x = ps0->pose.orientation.x;
    base_pose.pose.orientation.y = ps0->pose.orientation.z;
    base_pose.pose.orientation.z = ps0->pose.orientation.y;

    top_pose.pose.position.x = ps1->pose.position.x;
    top_pose.pose.position.y = ps1->pose.position.z;
    top_pose.pose.position.z = ps1->pose.position.y;

    top_pose.pose.orientation.w = ps1->pose.orientation.w;
    top_pose.pose.orientation.x = ps1->pose.orientation.x;
    top_pose.pose.orientation.y = ps1->pose.orientation.z;
    top_pose.pose.orientation.z = ps1->pose.orientation.y;

// Print out after mapping before sending
    // std::cout<<"\n Sample count is : "<<std::endl;
    // ROS_INFO("\n Position: \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n Orientation: \n \t w:[%f] \n \t x:[%f] \n \t y:[%f] \n \t z:[%f] \n",
    //           top_pose.pose.position.x,top_pose.pose.position.y,top_pose.pose.position.z,
    //           top_pose.pose.orientation.w,top_pose.pose.orientation.x,top_pose.pose.orientation.y,top_pose.pose.orientation.z
    // );

//Declare a ZMQ message queue  
    zmq::message_t message(140);
    
// Serialize (convert the data into a string) over zmq message AS PER THE ATT_POS_MOCAP_MSG PACK
    // snprintf((char *) message.data(),140, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    //             base_pose.pose.position.x,base_pose.pose.position.y,base_pose.pose.position.z,
    //             base_pose.pose.orientation.w,base_pose.pose.orientation.x,base_pose.pose.orientation.y,base_pose.pose.orientation.z,
    //             top_pose.pose.position.x,top_pose.pose.position.y,top_pose.pose.position.z,
    //             top_pose.pose.orientation.w,top_pose.pose.orientation.x,top_pose.pose.orientation.y,top_pose.pose.orientation.z);
    snprintf((char *) message.data(),140, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                ps0->pose.position.x,ps0->pose.position.y,ps0->pose.position.z,
                ps0->pose.orientation.w,ps0->pose.orientation.x,ps0->pose.orientation.y,ps0->pose.orientation.z,
                ps1->pose.position.x,ps1->pose.position.y,ps1->pose.position.z,
                ps1->pose.orientation.w,ps1->pose.orientation.x,ps1->pose.orientation.y,ps1->pose.orientation.z);
    
// Publish the message 0
    pub_sock.publisher.send(message);
    
// Frequency of publishing should be between 30-50 Hz. Currently set to 50 Hz.
    sleep(0.02); 
}

int main(int argc, char **argv)
{
// Initialize ROS and give a unique name for this node
    ros::init(argc,argv,"mocap_sub");

// Create a handle for this process' node
    ros::NodeHandle nh;
    
// Subscribe to the message of type "PoseStamped" published over the topic mentioned below in a buffer of size 1000
    message_filters::Subscriber<geometry_msgs::PoseStamped> top_sub(nh,"vrpn_client_node/RigidBody2/pose",1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> base_sub(nh,"vrpn_client_node/RigidBody1/pose",1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), base_sub, top_sub);
    sync.registerCallback(boost::bind(&Callback4, _1, _2));

    // ros::Subscriber sub = nh.subscribe("vrpn_client_node/top/pose",1000, &Callback);
    // ros::Subscriber sub2 = nh.subscribe("vrpn_client_node/base/pose",1000, &Callback2);
    // Keeps c++ from exiting until the node is stopped
    ros::spin();

    return 0;

}

