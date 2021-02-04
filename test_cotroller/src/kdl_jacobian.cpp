#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <iostream> 



int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "kdl_jacobian");
    ros::NodeHandle nh;

    // define kdl tree and chain for each arm
    KDL::Tree yumi_tree;
    KDL::Chain yumi_right_arm;
    KDL::Chain yumi_left_arm;

    // get tree from urdf file for entire yumi
    if (!kdl_parser::treeFromFile("/home/gabriel/catkin/src/test_cotroller/src/yumi.urdf", yumi_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    // get chain for each arm 
    yumi_tree.getChain("yumi_base_link","yumi_link_7_r", yumi_right_arm);
    yumi_tree.getChain("yumi_base_link","yumi_link_7_l", yumi_left_arm);

    // define jacobian for each arm
    KDL::Jacobian jacobian_right_arm(7);
    KDL::Jacobian jacobian_left_arm(7);

    std::cout << "Number of joints for yumi " << yumi_tree.getNrOfJoints() << std::endl;
    std::cout << "Number of joints for right arm " << yumi_right_arm.getNrOfJoints() << std::endl;
    std::cout << "Number of joints for left arm " << yumi_left_arm.getNrOfJoints() << std::endl;

    // Jacobian solver
    KDL::ChainJntToJacSolver jac_solver_right_arm(yumi_right_arm);
    KDL::ChainJntToJacSolver jac_solver_left_arm(yumi_left_arm);
    // joint values -- assuming same as defined in urdf
    KDL::JntArray q_right_arm(7);
    KDL::JntArray q_left_arm(7);

    // just a test for now
    for (int i= 0 ; i < 7; i++ ){
        q_right_arm(i) = 0;
        q_left_arm(i) = 0;
    }

    jac_solver_right_arm.JntToJac(q_right_arm, jacobian_right_arm);
    jac_solver_right_arm.JntToJac(q_left_arm, jacobian_left_arm);

    std::cout <<"Jacobian right (0,0) " << jacobian_right_arm(0,0) << std::endl;
    std::cout <<"Jacobian left (0,0) " << jacobian_left_arm(0,0) << std::endl;
    

    // for multithreading ros
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate loop_rate(0.5);

    uint32_t seq = 1;

    while (ros::ok()){
        ros::spinOnce();

        loop_rate.sleep();
        ++seq;
    }

    ros::waitForShutdown();

    return 0;
}