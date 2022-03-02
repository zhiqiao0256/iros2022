//Class for declaring default values of zmq publisher socket
#include<zmq.h>
#include"zhelpers.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"


class pub_socket{

		public :

    	zmq::context_t context;
    	zmq::socket_t publisher;
    	zmq::context_t sub_context;
    	zmq::socket_t subscriber;
    
    	const float rot_q[4];

    	pub_socket();  

    
 
	};

