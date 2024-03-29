/*
2014.10.29------------------------
このヘッダがあるだけでJSON ---> ROS Messageできるようにする
*/


#pragma
#include "picojson.h"
#include <humans_msgs/Humans.h>
//#include <tf/transform_broadcaster.h>
#include <cstdlib>

typedef std::map<int, std::string> JointMap;


class POSE{
public:
  double x;
  double y;
  double z;
};

namespace JsonToMsg{
  /*
  bool publishJointTF(ros::NodeHandle& nh, tf::TransformBroadcaster& br, tf::Transform& transform, std::string j_name, POSE j, std::string tf_prefix, int uid)
  {
    //cout << "joint name: "<< j_name << ", (x, y, z) = " <<j.position.x <<", "<<j.position.y <<", "<<j.position.z <<endl;
    transform.setOrigin(tf::Vector3(j.z ,j.x, j.y));
    //transform.setRotation(tf::Quaternion(j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    std::stringstream frame_id_stream;
    std::string frame_id;
    frame_id_stream << "/user_" << uid << "/" << j_name;
    frame_id = frame_id_stream.str();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_prefix, frame_id));
    return true;
  }
  */
  void body( const KinectPack kinectPack , humans_msgs::Humans *kinect_msg, double cols, double rows)
  {
    JointMap named_joints;
    named_joints[0] = "SpineBase";
    named_joints[1] = "SpineMid"; 
    named_joints[2] = "Neck";  
    named_joints[3] = "Head";
    named_joints[4] = "ShoulderLeft";
    named_joints[5] = "ElbowLeft";
    named_joints[6] = "WristLeft";
    named_joints[7] = "HandLeft";
    named_joints[8] = "ShoulderRight";
    named_joints[9] = "ElbowRight";
    named_joints[10] = "WristRight";
    named_joints[11] = "HandRight";
    named_joints[12] = "HipLeft";
    named_joints[13] = "KneeLeft";
    named_joints[14] = "AnkleLeft";
    named_joints[15] = "FootLeft";
    named_joints[16] = "HipRight";
    named_joints[17] = "KneeRight";
    named_joints[18] = "AnkleRight";
    named_joints[19] = "FootRight";
    named_joints[20] = "SpineShoulder"; 
    named_joints[21] = "HandTipLeft";
    named_joints[22] = "ThumbLeft";
    named_joints[23] = "HandTipRight";
    named_joints[24] = "ThumbRight";

    picojson::value v;
    std::string err;
    picojson::parse( v, kinectPack.bodies.jsonBodyInfo.begin(), kinectPack.bodies.jsonBodyInfo.end(), &err );


    if ( err.empty() )
      {
	picojson::object &objBodyInfo = v.get< picojson::object >();
	picojson::array arrayBody = objBodyInfo["bodies"].get< picojson::array >();
	//kinect_msg->human.resize(arrayBody.size());

	int index = 0;
	int people_num = 0;
	for( std::vector<picojson::value>::iterator itrBody = arrayBody.begin(); itrBody != arrayBody.end(); ++itrBody, ++index )
	  {
	    picojson::object &objBody = itrBody->get<picojson::object>();
	    bool isTracking =  objBody["isTracked"].get< bool >() ;
	    if ( isTracking )
	      {
		long long tracking_id = std::atoll( objBody["trackingId"].get< std::string >().c_str() );
		std::cout << "tracking_id[" << index << "]:" << tracking_id << std::endl;
		humans_msgs::Human tmp_human;
		//hoge.body.is_tracked = isTracking;
		
		tmp_human.body.is_tracked = isTracking;
	        tmp_human.body.tracking_id = tracking_id;
		tmp_human.body.left_hand_state = objBody["leftHandState"].get< double >() ;
	        tmp_human.body.right_hand_state = objBody["rightHandState"].get< double >() ;
		
		/*
		kinect_msg->human[index].body.is_tracked = isTracking;
		kinect_msg->human[index].body.tracking_id = tracking_id;
		kinect_msg->human[index].body.left_hand_state = objBody["leftHandState"].get< double >() ;
		kinect_msg->human[index].body.right_hand_state = objBody["rightHandState"].get< double >() ; 
		*/
		//kinect_msg->human.push_back( hoge );

		picojson::array arrayJoint = objBody["Joints"].get<picojson::array>();
		//kinect_msg->human[index].body.joints.resize(arrayJoint.size());
		int j_name = 0;
		for( std::vector<picojson::value>::iterator itrJoint = arrayJoint.begin(); itrJoint != arrayJoint.end(); ++itrJoint, ++j_name)
		  {
		    humans_msgs::Joints tmp_joint;
		    
		    picojson::object &objJoint = itrJoint->get<picojson::object>();
		    picojson::object &objPositionColorSpace = objJoint["PositionColorSpace"].get<picojson::object>();
		    tmp_joint.position_color_space.x = (int)objPositionColorSpace["X"].get<double>() * cols;
		    tmp_joint.position_color_space.y = (int)objPositionColorSpace["Y"].get<double>() * rows;
		    
		    picojson::object &objPosition = objJoint["Position"].get<picojson::object>();
		    tmp_joint.position.y = (double)objPosition["X"].get<double>();
		    tmp_joint.position.z = (double)objPosition["Y"].get<double>();
		    tmp_joint.position.x = (double)objPosition["Z"].get<double>();

		    picojson::object &objOrientation = objJoint["Orientation"].get<picojson::object>();
		    tmp_joint.orientation.x = (double)objOrientation["X"].get<double>();
		    tmp_joint.orientation.y = (double)objOrientation["Y"].get<double>();
		    tmp_joint.orientation.z = (double)objOrientation["Z"].get<double>();
		    tmp_joint.orientation.w = (double)objOrientation["W"].get<double>();

		    tmp_joint.tracking_state = objJoint["trackingState"].get<double>();
		    tmp_joint.joint_name = named_joints[j_name];

		    tmp_human.body.joints.push_back( tmp_joint );
		    //tmp_human.body.b
		    //おそらくBody_IndexはJSONにまだ含まれていない
		    //publishJointTF(nh, br, transform, named_joints[j_name], pose, tf_prefix, body_id);
		  }
		kinect_msg->human.push_back( tmp_human );	
		++people_num;
	      } 

	  }
	kinect_msg->num = people_num;
      }
    else
      {
	std::cerr << __FUNCTION__ << ":" << err << std::endl;
      } 
  } 
}
