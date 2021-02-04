#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

#include <apriltag_ros/AprilTagDetectionArray.h>


class Tf_camera_link
{
private:
    /* data */ 
    tf::Transform from_camera_to_tag = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform from_tag_to_camera = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0));
    ros::Subscriber apriltag_sub;
public:
    // constructor
    Tf_camera_link(ros::NodeHandle *nh );
    // Member functions declaration
    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tf_array_data);

    tf::Transform return_world_to_camera(){
        return from_tag_to_camera;
    }
};

// Member functions definitions
Tf_camera_link::Tf_camera_link(ros::NodeHandle *nh )
{
    apriltag_sub = nh->subscribe("/tag_detections", 2, &Tf_camera_link::callback, this);
}

void Tf_camera_link::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tf_array_data)
{   
    // check if element in list
    int num_elements = tf_array_data->detections.size();

    if (num_elements >= 1){

        for (int i = 0; i < num_elements; i++){
            // id 0 reserved for setting world origin.
            int index = tf_array_data->detections[i].id[0];
            //index = tf_array_data->detections[i].id[0];

            if (index == 0){
                double x = tf_array_data->detections[i].pose.pose.pose.position.x;
                double y = tf_array_data->detections[i].pose.pose.pose.position.y;
                double z = tf_array_data->detections[i].pose.pose.pose.position.z;

                double qx = tf_array_data->detections[i].pose.pose.pose.orientation.x;
                double qy = tf_array_data->detections[i].pose.pose.pose.orientation.y;
                double qz = tf_array_data->detections[i].pose.pose.pose.orientation.z;
                double qw = tf_array_data->detections[i].pose.pose.pose.orientation.w;

                from_camera_to_tag = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z));
                from_tag_to_camera = from_camera_to_tag.inverse();
            } 
        }
    }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;
    
    // for multithreading ros
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    Tf_camera_link tf_camera_link(&n);
    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;
    
    while(n.ok()){
        //ros::spinOnce();
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf_camera_link.return_world_to_camera(),
                ros::Time::now(),"world", "camera_link"));
        r.sleep();
    }
}
