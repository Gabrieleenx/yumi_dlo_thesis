#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <controller/Jacobian_msg.h>
#include <mutex>

// mutex 
std::mutex mtx_reciving;


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

void pose_data(KDL::Frame frame, geometry_msgs::Pose* pose){
    double Qx;
    double Qy;
    double Qz;
    double Qw;

    frame.M.GetQuaternion(Qx,Qy,Qz,Qw);

    pose->orientation.x = Qx;
    pose->orientation.y = Qy;
    pose->orientation.z = Qz;
    pose->orientation.w = Qw;

    pose->position.x = frame.p.data[0];
    pose->position.y = frame.p.data[1];
    pose->position.z = frame.p.data[2];
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
    KDL::Chain yumi_left_right;


    // define jacobian variables
    KDL::Jacobian jacobian_right_arm = KDL::Jacobian(7);
    KDL::Jacobian jacobian_left_arm = KDL::Jacobian(7);

    KDL::Jacobian jacobian_right_elbow = KDL::Jacobian(4);
    KDL::Jacobian jacobian_left_elbow = KDL::Jacobian(4);

    // define frame variables
    KDL::Frame frame_right_arm; 
    KDL::Frame frame_left_arm; 

    KDL::Frame frame_right_elbow; 
    KDL::Frame frame_left_elbow; 
    KDL::Frame frame_left_right; 

    // joint values -- assuming same as defined in urdf
    KDL::JntArray q_right_arm = KDL::JntArray(7);
    KDL::JntArray q_left_arm = KDL::JntArray(7);

    KDL::JntArray q_right_elbow = KDL::JntArray(4);
    KDL::JntArray q_left_elbow = KDL::JntArray(4);
 
    KDL::JntArray q_left_right = KDL::JntArray(14);

    // jacobian solvers
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_arm;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_arm;

    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_right_elbow;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_left_elbow;

    // forward kinematics solvers

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_right_arm;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_left_arm;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_left_to_right;

    //

    std::vector<double> joint_state;
    
    public:
    int state_recived = 0;

    // constructor
    Calc_jacobian(ros::NodeHandle *nh );

    void callback(const sensor_msgs::JointState::ConstPtr& joint_state_data);

    void update();
};

// Member functions definitions

Calc_jacobian::Calc_jacobian(ros::NodeHandle *nh ){
    joint_state_sub = nh->subscribe("/joint_states", 2, &Calc_jacobian::callback, this);
    //jacobian_pub = nh->advertise<std_msgs::Float64MultiArray>("/Jacobian_R_L", 2);
    jacobian_pub = nh->advertise<controller::Jacobian_msg>("/Jacobian_R_L", 1);
    
    // get tree from urdf file for entire yumi
    if (!kdl_parser::treeFromFile("/home/gabriel/catkin/src/yumi_dlo_thesis/yumi_description/urdf/yumi.urdf", yumi_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    
    // get chain for each arm 
    yumi_tree.getChain("yumi_base_link","yumi_link_7_r", yumi_right_arm);
    yumi_tree.getChain("yumi_base_link","yumi_link_7_l", yumi_left_arm);    

    yumi_tree.getChain("yumi_base_link","yumi_link_4_r", yumi_right_elbow);
    yumi_tree.getChain("yumi_base_link","yumi_link_4_l", yumi_left_elbow);  

    yumi_tree.getChain("yumi_link_7_l","yumi_link_7_r", yumi_left_right);    
  
    // Jacobian solver
    jac_solver_right_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_arm);
    jac_solver_left_arm = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_arm);

    jac_solver_right_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_right_elbow);
    jac_solver_left_elbow = std::make_unique<KDL::ChainJntToJacSolver>(yumi_left_elbow);

    // Frowark kinematics solver
    fk_solver_right_arm = std::make_unique<KDL::ChainFkSolverPos_recursive>(yumi_right_arm);
    fk_solver_left_arm = std::make_unique<KDL::ChainFkSolverPos_recursive>(yumi_left_arm);
    fk_left_to_right = std::make_unique<KDL::ChainFkSolverPos_recursive>(yumi_left_right);
    joint_state.resize(18);
}

void Calc_jacobian::callback(const sensor_msgs::JointState::ConstPtr& joint_state_data){
    //TODO To be updated 
    mtx_reciving.lock();
    std::copy(joint_state_data->position.begin(), joint_state_data->position.end(), joint_state.begin());
    //joint_state = (double)joint_state_data->position;
    state_recived = 1;
    mtx_reciving.unlock();

}

void Calc_jacobian::update(){

    mtx_reciving.lock();
   // joints state to q 
    for (int i= 0 ; i < 7; i++ ){
        q_right_arm(i) = joint_state[i];
        q_left_arm(i) = joint_state[i+7];
        q_left_right(i) = joint_state[i+7];
        q_left_right(i+7) = joint_state[i];
        if (i < 4){
            q_right_elbow(i) = joint_state[i];
            q_left_elbow(i) = joint_state[i+7];
        }
    }
    // --------------------- Jacobians --------------------------------------------------
    controller::Jacobian_msg jac_msg;
    // send joint position 
    jac_msg.jointPosition = joint_state;
    mtx_reciving.unlock();

    // arm 

    jac_solver_right_arm->JntToJac(q_right_arm, jacobian_right_arm);
    jac_solver_left_arm->JntToJac(q_left_arm, jacobian_left_arm);
    
    std_msgs::Float64MultiArray jac;

    jacobian_data(7, jacobian_right_arm, jacobian_left_arm, &jac);

    jac_msg.jacobian.push_back(jac);

    // elbow
    
    jac_solver_right_elbow->JntToJac(q_right_elbow, jacobian_right_elbow);
    jac_solver_left_elbow->JntToJac(q_left_elbow, jacobian_left_elbow);

    jacobian_data(4, jacobian_right_elbow, jacobian_left_elbow, &jac);

    jac_msg.jacobian.push_back(jac);

    jac_msg.header.stamp = ros::Time::now();

    // --------------------- Forward Kinematics --------------------------------------------------
    // last frame has number 8!
    fk_solver_right_arm->JntToCart(q_right_arm, frame_right_arm, 8);
    fk_solver_left_arm->JntToCart(q_left_arm, frame_left_arm, 8);

    fk_solver_right_arm->JntToCart(q_right_arm, frame_right_elbow, 5);
    fk_solver_left_arm->JntToCart(q_left_arm, frame_left_elbow, 5);

    fk_left_to_right->JntToCart(q_left_right, frame_left_right, -1);

    geometry_msgs::Pose pose;

    pose_data(frame_right_arm, &pose);
    jac_msg.forwardKinematics.push_back(pose);
 
    pose_data(frame_left_arm, &pose);
    jac_msg.forwardKinematics.push_back(pose);

    pose_data(frame_right_elbow, &pose);
    jac_msg.forwardKinematics.push_back(pose);

    pose_data(frame_left_elbow, &pose);
    jac_msg.forwardKinematics.push_back(pose);


    pose_data(frame_left_right, &pose);
    jac_msg.forwardKinematics.push_back(pose);

    jacobian_pub.publish(jac_msg);

    ros::spinOnce(); // sends msg 
}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "kdl_jacobian");
    ros::NodeHandle nh;
    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Calc_jacobian calc_jacobian(&nh);

    ros::Rate loop_rate(50);

    while (ros::ok()){
        if (calc_jacobian.state_recived >= 1){
            calc_jacobian.update();
        }
        loop_rate.sleep();
    }


    ros::waitForShutdown();

    return 0;
}