#include "ros/ros.h"
#include <iostream> 
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

// this rope/chain verlet simulation is inspierd by https://stackoverflow.com/questions/42609279/how-to-simulate-chain-physics-game-design/42618200


struct Parameters
{
    const double grav = -9.82;
    double groundHeight = 0;
    double airDrag = 0.97; // lower = more drag, 1 is no drag
    double contactDrag = 0.8;
    double stiffnes = 1; // stiffnes fo rope 
    const double deltaTime = 0.01;
    const double gravVelAdd = grav*deltaTime*deltaTime;
    const double holdFrictionSquared = 0.000005; // no idea whats a resonable value
};

struct Point
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};


struct Vertex
{
    Point pos;
    Point lastPos;
    bool fixed = false;
    double radius = 0.004;
    bool inContact = false;
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

struct RopeData
{
    std::vector<Point> points;
    std::vector<int> fixedIndices;
};


struct SimData
{
    std::vector<Vertex> points;
    std::vector<Link> links;
};


double calcNorm(Point* point0, Point* point1){
    double dx = point0->x - point1->x;
    double dy = point0->y - point1->y;
    double dz = point0->z - point1->z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


class RopeSim
{
private:
    /* data */
    SimData initialSimData; 
    SimData simData; 
    SimData simDataTemp; 
    SimData simDataTemp2; 
    Parameters parameters;
    ros::Publisher publish_rope;
    int case_ = 0;

public:
    RopeSim(ros::NodeHandle *nh);
    void updateInitial(RopeData* ropeData);
    void runSim();
    void movePoints();
    void pointConstraints();
    void resetToInitial();
    void linkConsstraints();
    void evaluate();
};

RopeSim::RopeSim(ros::NodeHandle *nh)
{
    /* constructor */
    publish_rope = nh->advertise<sensor_msgs::PointCloud>("/PointCloud", 1);

}

void RopeSim::updateInitial(RopeData* ropeData)
{
    /* update rope from DLO*/
    /* update map decribing the postiion of the fixtures */
    /* updates the target gripp points that are not reachable */
    int numPoints = ropeData->points.size();
    initialSimData.points.resize(numPoints);
    initialSimData.links.resize(numPoints-1);

    for (int i = 0; i < numPoints; i++)
    {
        Vertex point;
        point.pos = ropeData->points[i];
        point.lastPos = ropeData->points[i];

        for (int j = 0; j < ropeData->fixedIndices.size(); j++){
            if (i == ropeData->fixedIndices[j]){
                point.fixed = true;
                break;
            }
        }
        
        initialSimData.points[i] = point;

        if (i < numPoints-1){
            Link link;
            link.pointIndex0 = i;
            link.pointIndex1 = i+1;
            link.lineLength = calcNorm(&ropeData->points[i], &ropeData->points[i+1]);
            initialSimData.links[i] = link;
        }
    }


}

/*this might not be needed*/
void RopeSim::resetToInitial()
{
    /* resets the rope to DLO configuration */
    simData = initialSimData;
}

void RopeSim::movePoints(){
    /* move points due to gravity for non fixed points*/
    int numOfPoints = simData.points.size();
    double dx;
    double dy;
    double dz;
    Point meanPoint;
    Point meanPointTot;
    Point predPoint1;
    Point predPoint2;
    double length1;
    double length2;
   
    /* for the majority of points */
    for (int i = 2; i < (numOfPoints-2); i++)
    {
        if (simDataTemp.points[i].fixed == true){
            /* follow traj or fixture pos. */
            dx = 0;
            dy = 0;
            dz = 0;
        }
        else{
            /* Keep velocity */

            dx = (simDataTemp.points[i].pos.x - simDataTemp.points[i].lastPos.x);
            dy = (simDataTemp.points[i].pos.y - simDataTemp.points[i].lastPos.y);
            dz = (simDataTemp.points[i].pos.z - simDataTemp.points[i].lastPos.z);
            
            /* add gravity */
            dz += parameters.gravVelAdd;

            /* effeckts from stiffnes of rope */
            meanPoint.x = 0.5*(simDataTemp.points[i-1].pos.x + simDataTemp.points[i+1].pos.x);
            meanPoint.y = 0.5*(simDataTemp.points[i-1].pos.y + simDataTemp.points[i+1].pos.y);
            meanPoint.z = 0.5*(simDataTemp.points[i-1].pos.z + simDataTemp.points[i+1].pos.z);

            predPoint1 = simDataTemp.points[i-1].pos;
            length1 = simDataTemp.links[i-1].lineLength/simDataTemp.links[i-2].lineLength;
            predPoint1.x += (simDataTemp.points[i-1].pos.x - simDataTemp.points[i-2].pos.x) *length1;
            predPoint1.y += (simDataTemp.points[i-1].pos.y - simDataTemp.points[i-2].pos.y) *length1;
            predPoint1.z += (simDataTemp.points[i-1].pos.z - simDataTemp.points[i-2].pos.z) *length1;

            predPoint2 = simDataTemp.points[i+1].pos;
            length2 = simDataTemp.links[i].lineLength/simDataTemp.links[i+1].lineLength;
            predPoint2.x += (simDataTemp.points[i+1].pos.x - simDataTemp.points[i+2].pos.x) *length2;
            predPoint2.y += (simDataTemp.points[i+1].pos.y - simDataTemp.points[i+2].pos.y) *length2;
            predPoint2.z += (simDataTemp.points[i+1].pos.z - simDataTemp.points[i+2].pos.z) *length2;
            
            meanPointTot.x = 0.6 * meanPoint.x + 0.20 * predPoint1.x + 0.20 * predPoint2.x;
            meanPointTot.y = 0.6 * meanPoint.y + 0.20 * predPoint1.y + 0.20 * predPoint2.y;
            meanPointTot.z = 0.6 * meanPoint.z + 0.20 * predPoint1.z + 0.20 * predPoint2.z;

            dx += (meanPointTot.x - simDataTemp.points[i].pos.x)*parameters.stiffnes;
            dy += (meanPointTot.y - simDataTemp.points[i].pos.y)*parameters.stiffnes;
            dz += (meanPointTot.z - simDataTemp.points[i].pos.z)*parameters.stiffnes;

            /* drag */
            if (simDataTemp.points[i].inContact == false){
                dx = dx * parameters.airDrag;
                dy = dy * parameters.airDrag;
                dz = dz * parameters.airDrag; 
            }
            else{
                if (dx*dx+dy*dy < parameters.holdFrictionSquared){
                    dx = 0;
                    dy = 0;
                }
                dx = dx * parameters.contactDrag;
                dy = dy * parameters.contactDrag;
                dz = dz * parameters.contactDrag;
            }

            simData.points[i].pos.x += dx;
            simData.points[i].pos.y += dy;
            simData.points[i].pos.z += dz;
        }
    }

    /* for the rope ends */
    std::vector<int> endIndexList = {0,1,numOfPoints-2, numOfPoints-1};
    for (int j = 0; j < 4; j++)
    {
        int i = endIndexList[j];
        if (simDataTemp.points[i].fixed == true){
            /* follow traj or fixture pos. */
            dx = 0;
            dy = 0;
            dz = 0;
        }
        else{
            /* Keep velocity */

            dx = (simDataTemp.points[i].pos.x - simDataTemp.points[i].lastPos.x);
            dy = (simDataTemp.points[i].pos.y - simDataTemp.points[i].lastPos.y);
            dz = (simDataTemp.points[i].pos.z - simDataTemp.points[i].lastPos.z);
            
            /* add gravity */
            dz += parameters.gravVelAdd;

            /* effeckts from stiffnes of rope */
            if(j == 0){
                predPoint2 = simDataTemp.points[i+1].pos;
                length2 = simDataTemp.links[i].lineLength/simDataTemp.links[i+1].lineLength;
                predPoint2.x += (simDataTemp.points[i+1].pos.x - simDataTemp.points[i+2].pos.x) *length2;
                predPoint2.y += (simDataTemp.points[i+1].pos.y - simDataTemp.points[i+2].pos.y) *length2;
                predPoint2.z += (simDataTemp.points[i+1].pos.z - simDataTemp.points[i+2].pos.z) *length2;
                dx += (predPoint2.x - simDataTemp.points[i].pos.x)*parameters.stiffnes*0.20;
                dy += (predPoint2.y - simDataTemp.points[i].pos.y)*parameters.stiffnes*0.20;
                dz += (predPoint2.z - simDataTemp.points[i].pos.z)*parameters.stiffnes*0.20;
            }
            if(j == 1){
                meanPoint.x = 0.5*(simDataTemp.points[i-1].pos.x + simDataTemp.points[i+1].pos.x);
                meanPoint.y = 0.5*(simDataTemp.points[i-1].pos.y + simDataTemp.points[i+1].pos.y);
                meanPoint.z = 0.5*(simDataTemp.points[i-1].pos.z + simDataTemp.points[i+1].pos.z);
                predPoint2 = simDataTemp.points[i+1].pos;
                length2 = simDataTemp.links[i].lineLength/simDataTemp.links[i+1].lineLength;
                predPoint2.x += (simDataTemp.points[i+1].pos.x - simDataTemp.points[i+2].pos.x) *length2;
                predPoint2.y += (simDataTemp.points[i+1].pos.y - simDataTemp.points[i+2].pos.y) *length2;
                predPoint2.z += (simDataTemp.points[i+1].pos.z - simDataTemp.points[i+2].pos.z) *length2;
                meanPointTot.x = 0.5 * meanPoint.x + 0.5 * predPoint2.x;
                meanPointTot.y = 0.5 * meanPoint.y + 0.5 * predPoint2.y;
                meanPointTot.z = 0.5 * meanPoint.z + 0.5 * predPoint2.z;
                dx += (meanPointTot.x - simDataTemp.points[i].pos.x)*parameters.stiffnes*0.20;
                dy += (meanPointTot.y - simDataTemp.points[i].pos.y)*parameters.stiffnes*0.20;
                dz += (meanPointTot.z - simDataTemp.points[i].pos.z)*parameters.stiffnes*0.20;
            }
            if(j == 2){
                meanPoint.x = 0.5*(simDataTemp.points[i-1].pos.x + simDataTemp.points[i+1].pos.x);
                meanPoint.y = 0.5*(simDataTemp.points[i-1].pos.y + simDataTemp.points[i+1].pos.y);
                meanPoint.z = 0.5*(simDataTemp.points[i-1].pos.z + simDataTemp.points[i+1].pos.z);
                predPoint1 = simDataTemp.points[i-1].pos;
                length1 = simDataTemp.links[i-1].lineLength/simDataTemp.links[i-2].lineLength;
                predPoint1.x += (simDataTemp.points[i-1].pos.x - simDataTemp.points[i-2].pos.x) *length1;
                predPoint1.y += (simDataTemp.points[i-1].pos.y - simDataTemp.points[i-2].pos.y) *length1;
                predPoint1.z += (simDataTemp.points[i-1].pos.z - simDataTemp.points[i-2].pos.z) *length1;
                meanPointTot.x = 0.5 * meanPoint.x + 0.5 * predPoint1.x;
                meanPointTot.y = 0.5 * meanPoint.y + 0.5 * predPoint1.y;
                meanPointTot.z = 0.5 * meanPoint.z + 0.5 * predPoint1.z;
                dx += (meanPointTot.x - simDataTemp.points[i].pos.x)*parameters.stiffnes*0.20;
                dy += (meanPointTot.y - simDataTemp.points[i].pos.y)*parameters.stiffnes*0.20;
                dz += (meanPointTot.z - simDataTemp.points[i].pos.z)*parameters.stiffnes*0.20;
            }
            if (j == 3){
               predPoint1 = simDataTemp.points[i-1].pos;
                length1 = simDataTemp.links[i-1].lineLength/simDataTemp.links[i-2].lineLength;
                predPoint1.x += (simDataTemp.points[i-1].pos.x - simDataTemp.points[i-2].pos.x) *length1;
                predPoint1.y += (simDataTemp.points[i-1].pos.y - simDataTemp.points[i-2].pos.y) *length1;
                predPoint1.z += (simDataTemp.points[i-1].pos.z - simDataTemp.points[i-2].pos.z) *length1;
                dx += (predPoint1.x - simDataTemp.points[i].pos.x)*parameters.stiffnes*0.20;
                dy += (predPoint1.y - simDataTemp.points[i].pos.y)*parameters.stiffnes*0.20;
                dz += (predPoint1.z - simDataTemp.points[i].pos.z)*parameters.stiffnes*0.20;
            }

            /* drag */
            if (simDataTemp.points[i].inContact == false){
                dx = dx * parameters.airDrag;
                dy = dy * parameters.airDrag;
                dz = dz * parameters.airDrag; 
            }
            else{
                if (dx*dx+dy*dy < parameters.holdFrictionSquared){
                    dx = 0;
                    dy = 0;
                }
                dx = dx * parameters.contactDrag;
                dy = dy * parameters.contactDrag;
                dz = dz * parameters.contactDrag;
            }

            simData.points[i].pos.x += dx;
            simData.points[i].pos.y += dy;
            simData.points[i].pos.z += dz;

        }
    }


}

void RopeSim::pointConstraints(){
    /* constrainsts such as ground or fixtrues */
    int numOfPoints = simData.points.size();

    for(int i = 0; i < numOfPoints; i++){
        if (simData.points[i].pos.z <= 0.000001){
            simData.points[i].pos.z = 0;
            simData.points[i].inContact = 1;
        }
        else{
            simData.points[i].inContact = 0;
        }
    }
}

void RopeSim::linkConsstraints(){
    /* tries to keep the length between the points constant */
    int numOfLinks = simDataTemp2.links.size();
    int index0;
    int index1;
    double dist;
    double totalError;
    double factor;
    double scaler = 1;

    for (int i = 0; i < numOfLinks; i++){
        index0 = simDataTemp2.links[i].pointIndex0;
        index1 = simDataTemp2.links[i].pointIndex1;
        dist = calcNorm(&simDataTemp2.points[index0].pos, &simDataTemp2.points[index1].pos);

        totalError = simDataTemp2.links[i].lineLength - dist;
        if (simDataTemp2.points[index0].fixed == false && simDataTemp2.points[index1].fixed == false){
            factor =  scaler*0.5*totalError/dist;

            simData.points[index0].pos.x += factor * (simDataTemp2.points[index0].pos.x - simDataTemp2.points[index1].pos.x);
            simData.points[index0].pos.y += factor * (simDataTemp2.points[index0].pos.y - simDataTemp2.points[index1].pos.y);
            simData.points[index0].pos.z += factor * (simDataTemp2.points[index0].pos.z - simDataTemp2.points[index1].pos.z);

            simData.points[index1].pos.x += factor * (simDataTemp2.points[index1].pos.x - simDataTemp2.points[index0].pos.x);
            simData.points[index1].pos.y += factor * (simDataTemp2.points[index1].pos.y - simDataTemp2.points[index0].pos.y);
            simData.points[index1].pos.z += factor * (simDataTemp2.points[index1].pos.z - simDataTemp2.points[index0].pos.z);

        }
        else if (simDataTemp2.points[index0].fixed == true){
            factor =  scaler*totalError/dist;

            simData.points[index1].pos.x += factor * (simDataTemp2.points[index1].pos.x - simDataTemp2.points[index0].pos.x);
            simData.points[index1].pos.y += factor * (simDataTemp2.points[index1].pos.y - simDataTemp2.points[index0].pos.y);
            simData.points[index1].pos.z += factor * (simDataTemp2.points[index1].pos.z - simDataTemp2.points[index0].pos.z);

        }
        else if (simDataTemp2.points[index1].fixed == true){
            factor =  scaler*totalError/dist;

            simData.points[index0].pos.x += factor * (simDataTemp2.points[index0].pos.x - simDataTemp2.points[index1].pos.x);
            simData.points[index0].pos.y += factor * (simDataTemp2.points[index0].pos.y - simDataTemp2.points[index1].pos.y);
            simData.points[index0].pos.z += factor * (simDataTemp2.points[index0].pos.z - simDataTemp2.points[index1].pos.z);
        }
        else {

        }


    }
}

void RopeSim::evaluate(){
    /* evaluates the endstate fo the simulation and returns a score */
}

void RopeSim::runSim(){
    /* runs one simulation given trajectories for the grippers and outputs a score */ 

    simDataTemp = simData;
    if (case_ == 0){
        simData.points[30].pos.z += +0.0002;
        simData.points[40].pos.z += +0.0002;
        if(simData.points[30].pos.z > 0.1){
            case_ = 1;
        } 
        
    }
    else if (case_ == 1){
        simData.points[30].pos.y += +0.0002;
        simData.points[40].pos.y += +0.0002;
        if (simData.points[30].pos.y > 0.2){
            case_ = 2;
        }
    }
    else if (simData.points[30].pos.z > 0.0 && case_ == 2){
        simData.points[30].pos.z += -0.0002;
        simData.points[40].pos.z += -0.0002;
    }
    movePoints();

    pointConstraints();

    for (int i = 0; i < 100 ; i++){
        simDataTemp2 = simData;
        linkConsstraints();
    }
    
    for (int i = 0 ; i < simData.points.size(); i++){
        simData.points[i].lastPos = simDataTemp.points[i].pos;
    }
    
    sensor_msgs::PointCloud msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();

    geometry_msgs::Point32 point32;
    
    // PUBLISH MAP
    for(int i = 0; i < simData.points.size(); i++){
        point32.x = simData.points[i].pos.x;
        point32.y = simData.points[i].pos.y;
        point32.z = simData.points[i].pos.z;
        msg.points.push_back(point32);
    }
    publish_rope.publish(msg);
    
}




int main(int argc, char** argv){
    // ROS
    ros::init(argc, argv, "ropeSim");
    ros::NodeHandle nh;
    RopeSim ropeSim(&nh);
    RopeData ropeData;
    ropeData.points.resize(50);
    double wa = 0;
    for(int i = 0; i<50; i++){
        ropeData.points[i].x = wa;
        ropeData.points[i].y = 0;
        ropeData.points[i].z = 0;
        wa += 0.02;
    }
    ropeData.fixedIndices.resize(2);

    ropeData.fixedIndices[0] = 30;
    ropeData.fixedIndices[1] = 40;


    ropeSim.updateInitial(&ropeData);
    ropeSim.resetToInitial();
    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate loop_rate(100);
    while (ros::ok()){
        /*
        for (int j = 0; j < 100 ; j++){
            ropeSim.resetToInitial();
            for (int i =0 ; i < 500 ; i++){
                ropeSim.runSim();

            }
        }
        std::cout << "100 sims 10 sec each " << std::endl; 
        */
        ropeSim.runSim();
        loop_rate.sleep();
    }
    
    ros::waitForShutdown();

    return 0;
}