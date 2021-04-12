#include "ros/ros.h"
#include <iostream> 
#include <cmath>
#include <vector>
// this rope/chain verlet simulation is inspierd by https://stackoverflow.com/questions/42609279/how-to-simulate-chain-physics-game-design/42618200


struct Parameters
{
    const double grav = -9.82;
    double groundHeight = 0;
    double airDrag = 0.9; // lower = more drag, 1 is no drag
    double contactDrag = 0.1;
    double stiffnes = 0.01; // stiffnes fo rope 
    const double deltaTime = 0.02
    const double gravVelAdd = grav*deltaTime*deltaTime;
    const double holdFrictionSquared = 0.0001; // no idea whats a resonable value
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


class RopeSim
{
private:
    /* data */
    SimData initialSimData; 
    SimData simData; 
    SimData simDataTemp; 
    Parameters parameters;


public:
    RopeSim(/* args */);
    updateInitial(RopeData* ropeData);
    runSim();
    movePoints();

};

RopeSim::RopeSim(/* args */)
{
    /* constructor */
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

        for (int j = 0; j < ropeData->fixedindices.size(); j++){
            if (i == ropeData->fixedindices[j]){
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
    for (int i = 2; i < numOfPoints-2; i++)
    {
        if (simDataTemp.points[i].fixed == true){
            /* follow traj or fixture pos. */
            
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
            length1 = simDataTemp.link[i-1].lineLength/simDataTemp.link[i-2].lineLength;
            predPoint1.x += (simDataTemp.points[i-1].pos.x + simDataTemp.points[i-2].pos.x) *length1;
            predPoint1.y += (simDataTemp.points[i-1].pos.y + simDataTemp.points[i-2].pos.y) *length1;
            predPoint1.z += (simDataTemp.points[i-1].pos.z + simDataTemp.points[i-2].pos.z) *length1;

            predPoint2 = simDataTemp.points[i+1].pos;
            length2 = simDataTemp.link[i+1].lineLength/simDataTemp.link[i+2].lineLength;
            predPoint2.x += (simDataTemp.points[i+1].pos.x + simDataTemp.points[i+2].pos.x) *length2;
            predPoint2.y += (simDataTemp.points[i+1].pos.y + simDataTemp.points[i+2].pos.y) *length2;
            predPoint2.z += (simDataTemp.points[i+1].pos.z + simDataTemp.points[i+2].pos.z) *length2;
            
            meanPointTot.x = 0.5 * meanPoint.x + 0.25 * predPoint1.x + 0.25 * predPoint2.x;
            meanPointTot.y = 0.5 * meanPoint.y + 0.25 * predPoint1.y + 0.25 * predPoint2.y;
            meanPointTot.z = 0.5 * meanPoint.z + 0.25 * predPoint1.z + 0.25 * predPoint2.z;

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
                if (dx*dx+dy*dy+dz*dz < parameters.holdFrictionSquared){
                    dx = 0;
                    dy = 0;
                    dz = 0;
                }
                dx = dx * parameters.groundDrag;
                dy = dy * parameters.groundDrag;
                dz = dz * parameters.groundDrag;
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
                length2 = simDataTemp.link[i+1].lineLength/simDataTemp.link[i+2].lineLength;
                predPoint2.x += (simDataTemp.points[i+1].pos.x + simDataTemp.points[i+2].pos.x) *length2;
                predPoint2.y += (simDataTemp.points[i+1].pos.y + simDataTemp.points[i+2].pos.y) *length2;
                predPoint2.z += (simDataTemp.points[i+1].pos.z + simDataTemp.points[i+2].pos.z) *length2;
                dx += (predPoint2.x - simDataTemp.points[i].pos.x)*parameters.stiffnes;
                dy += (predPoint2.y - simDataTemp.points[i].pos.y)*parameters.stiffnes;
                dz += (predPoint2.z - simDataTemp.points[i].pos.z)*parameters.stiffnes;
            }
            if(j == 1){
                meanPoint.x = 0.5*(simDataTemp.points[i-1].pos.x + simDataTemp.points[i+1].pos.x);
                meanPoint.y = 0.5*(simDataTemp.points[i-1].pos.y + simDataTemp.points[i+1].pos.y);
                meanPoint.z = 0.5*(simDataTemp.points[i-1].pos.z + simDataTemp.points[i+1].pos.z);
                predPoint2 = simDataTemp.points[i+1].pos;
                length2 = simDataTemp.link[i+1].lineLength/simDataTemp.link[i+2].lineLength;
                predPoint2.x += (simDataTemp.points[i+1].pos.x + simDataTemp.points[i+2].pos.x) *length2;
                predPoint2.y += (simDataTemp.points[i+1].pos.y + simDataTemp.points[i+2].pos.y) *length2;
                predPoint2.z += (simDataTemp.points[i+1].pos.z + simDataTemp.points[i+2].pos.z) *length2;
                meanPointTot.x = 0.5 * meanPoint.x + 0.5 * predPoint2.x;
                meanPointTot.y = 0.5 * meanPoint.y + 0.5 * predPoint2.y;
                meanPointTot.z = 0.5 * meanPoint.z + 0.5 * predPoint2.z;
                dx += (meanPointTot.x - simDataTemp.points[i].pos.x)*parameters.stiffnes;
                dy += (meanPointTot.y - simDataTemp.points[i].pos.y)*parameters.stiffnes;
                dz += (meanPointTot.z - simDataTemp.points[i].pos.z)*parameters.stiffnes;
            }
            if(j == 2){
                meanPoint.x = 0.5*(simDataTemp.points[i-1].pos.x + simDataTemp.points[i+1].pos.x);
                meanPoint.y = 0.5*(simDataTemp.points[i-1].pos.y + simDataTemp.points[i+1].pos.y);
                meanPoint.z = 0.5*(simDataTemp.points[i-1].pos.z + simDataTemp.points[i+1].pos.z);
                predPoint1 = simDataTemp.points[i-1].pos;
                length1 = simDataTemp.link[i-1].lineLength/simDataTemp.link[i-2].lineLength;
                predPoint1.x += (simDataTemp.points[i-1].pos.x + simDataTemp.points[i-2].pos.x) *length1;
                predPoint1.y += (simDataTemp.points[i-1].pos.y + simDataTemp.points[i-2].pos.y) *length1;
                predPoint1.z += (simDataTemp.points[i-1].pos.z + simDataTemp.points[i-2].pos.z) *length1;
                meanPointTot.x = 0.5 * meanPoint.x + 0.5 * predPoint1.x;
                meanPointTot.y = 0.5 * meanPoint.y + 0.5 * predPoint1.y;
                meanPointTot.z = 0.5 * meanPoint.z + 0.5 * predPoint1.z;
                dx += (meanPointTot.x - simDataTemp.points[i].pos.x)*parameters.stiffnes;
                dy += (meanPointTot.y - simDataTemp.points[i].pos.y)*parameters.stiffnes;
                dz += (meanPointTot.z - simDataTemp.points[i].pos.z)*parameters.stiffnes;
            }
            if (j == 3){
               predPoint1 = simDataTemp.points[i-1].pos;
                length1 = simDataTemp.link[i-1].lineLength/simDataTemp.link[i-2].lineLength;
                predPoint1.x += (simDataTemp.points[i-1].pos.x + simDataTemp.points[i-2].pos.x) *length1;
                predPoint1.y += (simDataTemp.points[i-1].pos.y + simDataTemp.points[i-2].pos.y) *length1;
                predPoint1.z += (simDataTemp.points[i-1].pos.z + simDataTemp.points[i-2].pos.z) *length1;
                dx += (predPoint1.x - simDataTemp.points[i].pos.x)*parameters.stiffnes;
                dy += (predPoint1.y - simDataTemp.points[i].pos.y)*parameters.stiffnes;
                dz += (predPoint1.z - simDataTemp.points[i].pos.z)*parameters.stiffnes;
            }

            /* drag */
            if (simDataTemp.points[i].inContact == false){
                dx = dx * parameters.airDrag;
                dy = dy * parameters.airDrag;
                dz = dz * parameters.airDrag; 
            }
            else{
                if (dx*dx+dy*dy+dz*dz < parameters.holdFrictionSquared){
                    dx = 0;
                    dy = 0;
                    dz = 0;
                }
                dx = dx * parameters.groundDrag;
                dy = dy * parameters.groundDrag;
                dz = dz * parameters.groundDrag;
            }

            simData.points[i].pos.x += dx;
            simData.points[i].pos.y += dy;
            simData.points[i].pos.z += dz;
        }
    }


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

    simDataTemp = simData;
    movePoints();
    pointConstraints();



}


double calcNorm(Point* point0, Point* point1){
    double dx = point0->x - point1->x;
    double dy = point0->y - point1->y;
    double dz = point0->z - point1->z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
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