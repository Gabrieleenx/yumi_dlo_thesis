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
#include <abb_egm_msgs/EGMState.h>

#include <ros/package.h>

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
    ros::Subscriber egm_state_sub;

    ros::Publisher velocity_pub;
    ros::Publisher jacobian_pub;
    ros::Publisher joint_states_pub;

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

    // order input data

    std::vector<double> joint_state;
    
    std::string joint_name_list[18] = {"yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
         "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r", 
         "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l", 
         "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l", "gripper_r_joint",
         "gripper_r_joint_m", "gripper_l_joint", "gripper_l_joint_m"};
    
    std::string name_list[18] = {"yumi_robr_joint_1", "yumi_robr_joint_2", "yumi_robr_joint_3", "yumi_robr_joint_4",
         "yumi_robr_joint_5", "yumi_robr_joint_6", "yumi_robr_joint_7", "yumi_robl_joint_1", "yumi_robl_joint_2", "yumi_robl_joint_3",
         "yumi_robl_joint_4", "yumi_robl_joint_5", "yumi_robl_joint_6", "yumi_robl_joint_7"};
    //0.006109, -0.01012, -0.00733, -0.02775, 0.01396, 0.003665, -0.018325
    std::vector<double> joint_offset = {0.006109, -0.01012, -0.00733, -0.05175, 0.01396, 0.003665, -0.018325,
                                         -0.00421, -0.001571, -0.005759, -0.017977, -0.003491, 0.006632, -0.02142};

    // egm active for both arms
    bool egm_active = false;

    public:
    int state_recived = 0;

    // constructor
    Calc_jacobian(ros::NodeHandle *nh );

    void callback(const sensor_msgs::JointState::ConstPtr& joint_state_data);

    void callback_egm_state(const abb_egm_msgs::EGMState::ConstPtr& egm_state_data);


    void update();
};

// Member functions definitions

Calc_jacobian::Calc_jacobian(ros::NodeHandle *nh ){
    joint_state_sub = nh->subscribe("/yumi/egm/joint_states", 2, &Calc_jacobian::callback, this);
    egm_state_sub = nh->subscribe("/yumi/egm/egm_states", 2, &Calc_jacobian::callback_egm_state, this);

    //jacobian_pub = nh->advertise<std_msgs::Float64MultiArray>("/Jacobian_R_L", 2);
    jacobian_pub = nh->advertise<controller::Jacobian_msg>("/Jacobian_R_L", 1);
    joint_states_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
    velocity_pub = nh->advertise<std_msgs::Float64MultiArray>("/yumi/egm/joint_group_velocity_controller/command", 1);

    // get tree from urdf file for entire yumi
    std::string const PATH = ros::package::getPath("yumi_description");
    
    if (!kdl_parser::treeFromFile(PATH + "/urdf/yumi.urdf", yumi_tree)){
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
    // sort input data

    mtx_reciving.lock();
    for (int i = 0; i < 14; i++){
        for (int j = 0; j < 14; j++){
            if (name_list[i].compare(joint_state_data->name[j]) == 0 ){
                joint_state[i] = joint_state_data->position[j] + joint_offset[i];
                break;
            }
        }
    }
    int num_elements = joint_state_data->position.size();
    if (num_elements > 14){ // for simulation
        joint_state[14] = joint_state_data->position[14];
        joint_state[15] = joint_state_data->position[15];
        joint_state[16] = joint_state_data->position[16];
        joint_state[17] = joint_state_data->position[17];      
    }
    else{ // real robot do not give values for gripper position 
        joint_state[14] = 0.0;
        joint_state[15] = 0.0;
        joint_state[16] = 0.0;
        joint_state[17] = 0.0;
    }
    state_recived = 1;
    mtx_reciving.unlock();
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 18; i++){
        msg.name.push_back(joint_name_list[i]);
        msg.position.push_back(joint_state[i]);
    }
   
    joint_states_pub.publish(msg);

}


void Calc_jacobian::callback_egm_state(const abb_egm_msgs::EGMState::ConstPtr& egm_state_data){
    if (egm_state_data->egm_channels[0].active == true && egm_state_data->egm_channels[1].active == true){
        egm_active = true;
    }
    else{
        egm_active = false;
    }
}


void Calc_jacobian::update(){

    if (egm_active == false){
        std_msgs::Float64MultiArray msg;
        std::vector<double> msg_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        msg.data = msg_data;
        velocity_pub.publish(msg);
        return;
    }

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


    pose_data(frame_left_right, &pose); // not in use ?
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