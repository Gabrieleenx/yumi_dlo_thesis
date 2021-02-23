#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <iostream> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller/Jacobian_msg.h>



void jacobian_data(int dof, KDL::Jacobian jacobian_right, KDL::Jacobian jacobian_left, std_msgs::Float64MultiArray* jac){
    std::vector<double> data_msg(6*dof*2); // 6*7*2

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < dof; j++){
            for(int k = 0; k <2; k++){
                data_msg[ 2*dof*i + 2*j + k] = (k == 0) ? jacobian_right(i,j):jacobian_left(i,j);
            }
        }
    }
    
    std_msgs::MultiArrayLayout msg_layout;
    std::vector<std_msgs::MultiArrayDimension> msg_dim(3);
    msg_dim[0].label = "height";
    msg_dim[0].size = 6;
    msg_dim[0].stride = 6*dof*2;
    msg_dim[1].label = "width";
    msg_dim[1].size = dof;
    msg_dim[1].stride = 2*dof;
    msg_dim[2].label = "channel";
    msg_dim[2].size = 2;
    msg_dim[2].stride = 2;

    msg_layout.dim = msg_dim;
    msg_layout.data_offset = 0;

    jac->layout = msg_layout;
    jac->data = data_msg;
}

class Calc_jacobian{
    private:
    ros::Subscriber joint_state_sub;
    ros::Publisher jacobian_pub;
    // define kdl tree and chain for each arm
    KDL::Tree yumi_tree;
    KDL::Chain yumi_right_arm;
    KDL::Chain yumi_left_arm;
    
    KDL::Chain yumi_right_elbow;
    KDL::Chain yumi_left_elbow;

    // define jacobian for each arm
    KDL::Jacobian jacobian_right_arm = KDL::Jacobian(7);
    KDL::Jacobian jacobian_left_arm = KDL::Jacobian(7);

    KDL::Jacobian jacobian_right_elbow = KDL::Jacobian(4);
    KDL::Jacobian jacobian_left_elbow = KDL::Jacobian(4);

    // joint values -- assuming same as defined in urdf
    KDL::JntArray q_right_arm = KDL::JntArray(7);
    KDL::JntArray q_left_arm = KDL::JntArray(7);

    KDL::JntArray q_right_elbow = KDL::JntArray(4);
    KDL::JntArray q_left_elbow = KDL::JntArray(4);

    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_arm;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_arm;

    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_elbow;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_elbow;

    public:
    // constructor
    Calc_jacobian(ros::NodeHandle *nh );

    void callback(const sensor_msgs::JointState::ConstPtr& joint_state_data);

};

// Member functions definitions

Calc_jacobian::Calc_jacobian(ros::NodeHandle *nh ){
    joint_state_sub = nh->subscribe("/joint_states", 2, &Calc_jacobian::callback, this);
    //jacobian_pub = nh->advertise<std_msgs::Float64MultiArray>("/Jacobian_R_L", 2);
    jacobian_pub = nh->advertise<controller::Jacobian_msg>("/Jacobian_R_L", 2);
    
    // get tree from urdf file for entire yumi
    if (!kdl_parser::treeFromFile("/home/gabriel/catkin/src/yumi_dlo_thesis/yumi_description/urdf/yumi.urdf", yumi_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    
        // get chain for each arm 
    yumi_tree.getChain("yumi_base_link","yumi_link_7_r", yumi_right_arm);
    yumi_tree.getChain("yumi_base_link","yumi_link_7_l", yumi_left_arm);    

    yumi_tree.getChain("yumi_base_link","yumi_link_4_r", yumi_right_elbow);
    yumi_tree.getChain("yumi_base_link","yumi_link_4_l", yumi_left_elbow);    
    // Jacobian solver
    jac_solver_right_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_arm);
    jac_solver_left_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_arm);

    jac_solver_right_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_elbow);
    jac_solver_left_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_elbow);
     
}

void Calc_jacobian::callback(const sensor_msgs::JointState::ConstPtr& joint_state_data){

    // joints state to q 
    for (int i= 0 ; i < 7; i++ ){
        q_right_arm(i) = joint_state_data->position[i];
        q_left_arm(i) = joint_state_data->position[i+7];

        if (i < 4){
            q_right_elbow(i) = joint_state_data->position[i];
            q_left_elbow(i) = joint_state_data->position[i+7];
        }
    }

    controller::Jacobian_msg jac_msg;
    
    // arm 

    jac_solver_right_arm->JntToJac(q_right_arm, jacobian_right_arm);
    jac_solver_left_arm->JntToJac(q_left_arm, jacobian_left_arm);
    
    std_msgs::Float64MultiArray jac;

    jacobian_data(7, jacobian_right_arm, jacobian_left_arm, &jac);

    jac_msg.jacobian.push_back(jac);

    // elbow
    
    jac_solver_right_arm->JntToJac(q_right_elbow, jacobian_right_elbow);
    jac_solver_left_arm->JntToJac(q_left_elbow, jacobian_left_elbow);

    jacobian_data(4, jacobian_right_arm, jacobian_left_arm, &jac);

    jac_msg.jacobian.push_back(jac);

    jacobian_pub.publish(jac_msg);
    ros::spinOnce(); // sends msg 
}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "kdl_jacobian");
    ros::NodeHandle nh;

    Calc_jacobian calc_jacobian(&nh);

    ros::spin();

    ros::waitForShutdown();

    return 0;
}