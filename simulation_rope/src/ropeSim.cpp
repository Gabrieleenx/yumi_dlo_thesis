#include "ros/ros.h"
#include <iostream> 
#include <cmath>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>

#include "ropeSim.h"
// this rope/chain verlet simulation is inspierd by https://stackoverflow.com/questions/42609279/how-to-simulate-chain-physics-game-design/42618200


class RopeSim
{
private:
    /* data */
    SimData initialSimData; 
    SimData simData; 
    SimData simDataTemp; 
    SimData simDataTemp2; 
    Parameters parameters;
    Grippers grippers;
    Fixture fixtures;
    ros::Publisher publish_rope;
    ros::Subscriber grippers_sub;
    tf::TransformBroadcaster broadcaster_fixtures;
    tf::TransformListener listener;
    tf::StampedTransform transformRightGripper;
    tf::StampedTransform transformLeftGripper;
    int grippedPointRight = 0;
    int grippedPointLeft = 0;
    int case_ = 0;

public:
    RopeSim(ros::NodeHandle *nh);
    void updateInitial(RopeData* ropeData, Fixture* fixtureData);
    void runSim();
    void movePoints();
    void pointConstraints();
    void resetToInitial();
    void linkConsstraints();
    void grippers_callback(const std_msgs::Float64MultiArray::ConstPtr& gripperData);
};

// constructor
RopeSim::RopeSim(ros::NodeHandle *nh)
{
    /* constructor */
    publish_rope = nh->advertise<sensor_msgs::PointCloud>("/spr/dlo_estimation", 1);
    grippers_sub = nh->subscribe("/sim/grippers", 2, &RopeSim::grippers_callback, this);

}

// member functions 

void RopeSim::grippers_callback(const std_msgs::Float64MultiArray::ConstPtr& gripperData){
    grippers.gripperRight = gripperData->data[0];
    grippers.gripperLeft = gripperData->data[1];
}

void RopeSim::updateInitial(RopeData* ropeData, Fixture* fixtureData)
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
    
    fixtures = *fixtureData;
}

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
    double dist; 
    Point point;
    double dx;
    double dy;
    double factor;
    for(int i = 0; i < numOfPoints; i++){
        if (simData.points[i].pos.z <= 0.000001){
            simData.points[i].pos.z = 0;
            simData.points[i].inContact = 1;
        }
        else{
            simData.points[i].inContact = 0;
        }
        if (simData.points[i].pos.z < 0.05){
            for (int j = 0; j < fixtures.numFixtures; j ++){
                dx = simData.points[i].pos.x - fixtures.fixtureList[j].getOrigin().x();
                dy = simData.points[i].pos.y - fixtures.fixtureList[j].getOrigin().y();
                dist = std::sqrt(dx*dx + dy*dy);
                if (dist <= fixtures.fixtureRadius){
                    if (dist == 0){
                        continue;
                    }
                    else{
                        factor = fixtures.fixtureRadius/dist;
                        simData.points[i].pos.x = fixtures.fixtureList[j].getOrigin().x() + dx*factor;
                        simData.points[i].pos.y = fixtures.fixtureList[j].getOrigin().y() + dy*factor;
                    }
                    simData.points[i].inContact = 1;
                }
            }
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


void RopeSim::runSim(){
    /* runs one simulation given trajectories for the grippers and outputs a score */ 
    try
    {
        listener.lookupTransform("/yumi_base_link", "/yumi_gripp_r",  
                                  ros::Time(0), transformRightGripper);
        listener.lookupTransform("/yumi_base_link", "/yumi_gripp_l",  
                                  ros::Time(0), transformLeftGripper);
    }
    catch(const std::exception& e)
    {
        std::cout << "Waiting for transform to ba available " << std::endl;
        ros::Duration(0.5).sleep();

    }
    
    
    
    double minDistRight;
    int minIndexRight;
    closestPoint(&transformRightGripper.getOrigin(), &simData, &minDistRight, &minIndexRight);
    double minDistLeft;
    int minIndexLeft;
    closestPoint(&transformLeftGripper.getOrigin(), &simData, &minDistLeft, &minIndexLeft);

    if(minDistRight < 0.015 && grippers.gripperRight <= 5.0){
        simData.points[minIndexRight].fixed = true;
        Point point;
        point.x = transformRightGripper.getOrigin().x();
        point.y = transformRightGripper.getOrigin().y();
        point.z = transformRightGripper.getOrigin().z();
        simData.points[minIndexRight].pos = point;
        grippedPointRight = minIndexRight;
    }
    else if (minDistRight < 0.02 && grippers.gripperRight >= 6.0){
        simData.points[grippedPointRight].fixed = false;
    }

    if(minDistLeft < 0.015 && grippers.gripperLeft <= 5.0){
        simData.points[minIndexLeft].fixed = true;
        Point point;
        point.x = transformLeftGripper.getOrigin().x();
        point.y = transformLeftGripper.getOrigin().y();
        point.z = transformLeftGripper.getOrigin().z();
        simData.points[minIndexLeft].pos = point;
        grippedPointLeft = minIndexLeft;
    }
    else if (minDistLeft < 0.02 && grippers.gripperLeft >= 6.0){
        simData.points[grippedPointLeft].fixed = false;
    }

    double minDistfixture;
    int minIndexfixtue;
    for (int i = 0; i<fixtures.numFixtures; i++){
        tf::Vector3 fixtureClipp;
        fixtureClipp.setX(fixtures.fixtureList[i].getOrigin().x());
        fixtureClipp.setY(fixtures.fixtureList[i].getOrigin().y());
        fixtureClipp.setZ(fixtures.fixtureList[i].getOrigin().z() + 0.06);

        closestPoint(&fixtureClipp, &simData, &minDistfixture, &minIndexfixtue);
        
        if(minDistfixture < 0.010){
            simData.points[minIndexfixtue].fixed = true;
            simData.points[minIndexfixtue-1].fixed = true;
            simData.points[minIndexfixtue+1].fixed = true;
        }
    }

    simDataTemp = simData;

    movePoints();

    pointConstraints();

    for (int i = 0; i < 200 ; i++){
        simDataTemp2 = simData;
        linkConsstraints();
    }
    
    for (int i = 0 ; i < simData.points.size(); i++){
        simData.points[i].lastPos = simDataTemp.points[i].pos;
    }
    
    sensor_msgs::PointCloud msg;
    msg.header.frame_id = "/yumi_base_link";
    msg.header.stamp = ros::Time::now();

    geometry_msgs::Point32 point32;
    
    // PUBLISH ROPE
    for(int i = 0; i < simData.points.size(); i++){
        point32.x = simData.points[i].pos.x;
        point32.y = simData.points[i].pos.y;
        point32.z = simData.points[i].pos.z;
        msg.points.push_back(point32);
    }
    publish_rope.publish(msg);

    for (int i = 0; i < fixtures.numFixtures; i++){
        if (fixtures.fixtureActive[i] == 1){
            broadcaster_fixtures.sendTransform(
                tf::StampedTransform(
                    fixtures.fixtureList[i],
                    ros::Time::now(), "yumi_base_link", fixtures.fixtureName[i]));
            }
    }
    
}


int main(int argc, char** argv){
    // ROS 
    // sim works /yumi_base_link and not in \world frame! 

    ros::init(argc, argv, "ropeSim");
    ros::NodeHandle nh;

    RopeSim ropeSim(&nh);

    RopeData ropeData;
    ropeData.points.resize(70);
    double wa = -0.35;
    for(int i = 0; i<70; i++){
        ropeData.points[i].x = 0.4;
        ropeData.points[i].y = wa;
        ropeData.points[i].z = 0;
        wa += 0.014285;
    }

    ropeData.fixedIndices.resize(0);

    Fixture fixture;

    tf::Quaternion quaternion;
    quaternion.setEulerZYX(0.0, 0.0, 0.0);
    fixture.fixtureList[0] = tf::Transform(quaternion, tf::Vector3(0.3, -0.2, 0.0));
    fixture.fixtureActive[0] = 1;

    quaternion.setEulerZYX(-45.0/180*M_PI, 0.0, 0.0);
    fixture.fixtureList[1] = tf::Transform(quaternion, tf::Vector3(0.25, -0.0, 0.0));
    fixture.fixtureActive[1] = 1;

    quaternion.setEulerZYX(0.0/180*M_PI, 0.0, 0.0);
    fixture.fixtureList[2] = tf::Transform(quaternion, tf::Vector3(0.3, 0.2, 0.0));
    fixture.fixtureActive[2] = 1;

    ropeSim.updateInitial(&ropeData, &fixture);
    ropeSim.resetToInitial();

    // multithreaded spinner 
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Duration(0.5).sleep();
    ros::Rate loop_rate(100);

    while (ros::ok()){
        ropeSim.runSim();
        loop_rate.sleep();
    }
    
    ros::waitForShutdown();

    return 0;
}