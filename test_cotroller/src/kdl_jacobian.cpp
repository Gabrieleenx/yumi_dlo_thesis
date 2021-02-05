#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <iostream> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


class Calc_jacobian{
    private:
    ros::Subscriber joint_state_sub;
    ros::Publisher jacobian_pub;
    // define kdl tree and chain for each arm
    KDL::Tree yumi_tree;
    KDL::Chain yumi_right_arm;
    KDL::Chain yumi_left_arm;

    
    // define jacobian for each arm
    KDL::Jacobian jacobian_right_arm = KDL::Jacobian(7);
    KDL::Jacobian jacobian_left_arm = KDL::Jacobian(7);

    // joint values -- assuming same as defined in urdf
    KDL::JntArray q_right_arm = KDL::JntArray(7);
    KDL::JntArray q_left_arm = KDL::JntArray(7);

    //bool from_urdf = !kdl_parser::treeFromFile("/home/gabriel/catkin/src/yumi_dlo_thesis/yumi_description/urdf/yumi.urdf", yumi_tree);
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_arm;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_arm;


    public:
    // constructor
    Calc_jacobian(ros::NodeHandle *nh );

    void callback(const sensor_msgs::JointState::ConstPtr& joint_state_data);


};

// Member functions definitions

Calc_jacobian::Calc_jacobian(ros::NodeHandle *nh ){
    joint_state_sub = nh->subscribe("/joint_states", 2, &Calc_jacobian::callback, this);
    jacobian_pub = nh->advertise<std_msgs::Float64MultiArray>("/Jacobian_R_L", 2);

    // get tree from urdf file for entire yumi
    if (!kdl_parser::treeFromFile("/home/gabriel/catkin/src/yumi_dlo_thesis/yumi_description/urdf/yumi.urdf", yumi_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    
        // get chain for each arm 
    yumi_tree.getChain("yumi_base_link","yumi_link_7_r", yumi_right_arm);
    yumi_tree.getChain("yumi_base_link","yumi_link_7_l", yumi_left_arm);    
    // Jacobian solver
    jac_solver_right_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_arm);
    jac_solver_left_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_arm);
    //KDL::ChainJntToJacSolver jac_solver_left_arm = KDL::ChainJntToJacSolver(yumi_left_arm);
     
}

void Calc_jacobian::callback(const sensor_msgs::JointState::ConstPtr& joint_state_data){

    // joints state to q 
    for (int i= 0 ; i < 7; i++ ){
        q_right_arm(i) = joint_state_data->position[i];
        q_left_arm(i) = joint_state_data->position[i+7];
    }

    jac_solver_right_arm->JntToJac(q_right_arm, jacobian_right_arm);
    jac_solver_right_arm->JntToJac(q_left_arm, jacobian_left_arm);

    std_msgs::Float64MultiArray jac_msg;
    std::vector<double> data_msg(84); // 6*7*2

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 7; j++){
            for(int k = 0; k <2; k++){
                data_msg[ 2*7*i + 2*j + k] = (k == 0) ? jacobian_right_arm(i,j):jacobian_left_arm(i,j);
            }
        }
    }
    
    std_msgs::MultiArrayLayout msg_layout;
    std::vector<std_msgs::MultiArrayDimension> msg_dim(3);
    msg_dim[0].label = "height";
    msg_dim[0].size = 6;
    msg_dim[0].stride = 6*7*2;
    msg_dim[1].label = "width";
    msg_dim[1].size = 7;
    msg_dim[1].stride = 2*7;
    msg_dim[2].label = "channel";
    msg_dim[2].size = 2;
    msg_dim[2].stride = 2;

    msg_layout.dim = msg_dim;
    msg_layout.data_offset = 0;

    jac_msg.layout = msg_layout;
    jac_msg.data = data_msg;
    jacobian_pub.publish(jac_msg);
    ros::spinOnce(); //maybe not neccecery but might decrease delay time

}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "kdl_jacobian");
    ros::NodeHandle nh;

    // for multithreading ros
    //ros::AsyncSpinner spinner(0);
    //spinner.start();

    Calc_jacobian calc_jacobian(&nh);

    ros::spin();

    ros::waitForShutdown();

    return 0;
}