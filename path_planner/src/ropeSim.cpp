#include "ros/ros.h"
#include <iostream> 

// this rope/chain verlet simulation is inspierd by https://stackoverflow.com/questions/42609279/how-to-simulate-chain-physics-game-design/42618200


struct Parameters
{
    double grav = -9.82;
    double groundHeight = 0;
    double airDrag = 0.1;
    double groundDrag = 0.9;
    double deltaTime = 0.02
};


struct Vertex
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double lastX = 0.0;
    double lastY = 0.0;
    double lastZ = 0.0;
    bool fixed = false;
    double radius = 0.004;
};

struct Link
{
    int pointIndex0 = 0;
    int pointIndex1 = 1;
    double lineLength = 0.01;
};

struct Fixture
{
    /* data */
};


class RopeSim
{
private:
    /* data */
    std::vector<Vertex> inititalPoints; 
public:
    RopeSim(/* args */);
    updateInitial();
};

RopeSim::RopeSim(/* args */)
{
    /* constructor */
}

void RopeSim::updateInitial(/*some intput*/)
{
    /* update rope from DLO*/
    /* update map decribing the postiion of the fixtures */
    /* updates the target gripp points that are not reachable */
    

}

/*this might not be needed*/
void RopeSim::resetToInitial()
{
    /* resets the rope to DLO configuration */
}

void RopeSim::moveGravity(){
    /* move points due to gravity for non fixed points*/
}

void RopeSim::pointConstraints(){
    /* constrainsts such as ground or fixtrues */
}

void RopeSim::linkConsstraints(){
    /* tries to keep the length between the points constant */
}

void RopeSim::evaluate(){
    /* evaluates the endstate fo the simulation and returns a score */
}

void RopeSim::runSim(){
    /* runs one simulation given trajectories for the grippers and outputs a score */ 
}


int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "ropeSim");
    ros::NodeHandle nh;
    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();

    
    ros::waitForShutdown();

    return 0;
}