#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

#include <apriltag_ros/AprilTagDetectionArray.h>

struct Fixture
{
    const static int numFixtures = 5;
    tf::Transform fixtureList[numFixtures] = {tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0))};
    int fixtureActive[numFixtures] = {0};
    std::string fixtureName[numFixtures] = {"Fixture1", "Fixture2", "Fixture3", "Fixture4", "Fixture5"};
};



class Tf_camera_link
{
private:
    /* data */ 
    tf::Transform from_camera_to_tag = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform from_tag_to_camera = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform from_camera_to_fixture = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0));
    tf::Transform from_tag_to_fixture = tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(-0.04, 0.0, 0.0));

    Fixture fixture;
    ros::Subscriber apriltag_sub;
public:
    // constructor
    Tf_camera_link(ros::NodeHandle *nh );
    // Member functions declaration
    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tf_array_data);

    tf::Transform return_world_to_camera(){
        return from_tag_to_camera;
    }

    Fixture returnFixtures(){
        return fixture;
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
            double x = tf_array_data->detections[i].pose.pose.pose.position.x;
            double y = tf_array_data->detections[i].pose.pose.pose.position.y;
            double z = tf_array_data->detections[i].pose.pose.pose.position.z;

            double qx = tf_array_data->detections[i].pose.pose.pose.orientation.x;
            double qy = tf_array_data->detections[i].pose.pose.pose.orientation.y;
            double qz = tf_array_data->detections[i].pose.pose.pose.orientation.z;
            double qw = tf_array_data->detections[i].pose.pose.pose.orientation.w;

            
            if (index == 0){
                from_camera_to_tag = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z));
                from_tag_to_camera = from_camera_to_tag.inverse();
            } 
            else{
                from_camera_to_tag = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z));
                from_camera_to_fixture = from_camera_to_tag*from_tag_to_fixture;
                fixture.fixtureActive[index-1] = 1;
                fixture.fixtureList[index-1] = from_camera_to_fixture;
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
    Fixture fixture;
    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    tf::TransformBroadcaster broadcaster2;
    tf::TransformBroadcaster broadcaster_r_gripper;
    tf::TransformBroadcaster broadcaster_l_gripper;
    tf::TransformBroadcaster broadcaster_fixtures;

    while(n.ok()){
        //ros::spinOnce();
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf_camera_link.return_world_to_camera(),
                ros::Time::now(),"world", "camera_link"));

        broadcaster2.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0.7071, -0.7071), tf::Vector3(0.181, 0, 0)),
                ros::Time::now(), "yumi_base_link", "world"));

        broadcaster_r_gripper.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.136)),
                ros::Time::now(), "yumi_link_7_r", "yumi_gripp_r"));

        broadcaster_l_gripper.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.136)),
                ros::Time::now(), "yumi_link_7_l", "yumi_gripp_l"));

        fixture = tf_camera_link.returnFixtures();
        for (int i=0; i < fixture.numFixtures; i++){
            if (fixture.fixtureActive[i] == 1){
                broadcaster_fixtures.sendTransform(
                    tf::StampedTransform(
                        fixture.fixtureList[i],
                        ros::Time::now(), "camera_link", fixture.fixtureName[i])
                );
            }
        }
        r.sleep();
    }
}
