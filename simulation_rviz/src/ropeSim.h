#pragma once

struct Fixture
{
    const static int numFixtures = 4;
    tf::Transform fixtureList[numFixtures] = {tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0))};
    int fixtureActive[numFixtures] = {0};
    std::string fixtureName[numFixtures] = {"Fixture1", "Fixture2", "Fixture3", "Fixture4"};
    double fixtureRadius = 0.03; // for collission
};

struct Parameters
{
    const double grav = -9.82;
    double groundHeight = 0;
    double airDrag = 0.90; // lower = more drag, 1 is no drag
    double contactDrag = 0.2;
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


struct Grippers
{
    double gripperRight = 0.0;
    double gripperLeft = 0.0;
};


double calcNorm(Point* point0, Point* point1){
    double dx = point0->x - point1->x;
    double dy = point0->y - point1->y;
    double dz = point0->z - point1->z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


void closestPoint(tf::Vector3* position, SimData* simData, double* minDist, int* minIndex){
    Point point;
    point.x = position->x();
    point.y = position->y();
    point.z = position->z();
    double dist;
    for (int i = 0; i < simData->points.size(); i++){
        dist = calcNorm(&point, &simData->points[i].pos);
        if (i == 0){
            *minDist = dist;
            *minIndex = i;
        }
        else if (dist < *minDist){
            *minDist = dist;
            *minIndex = i;
        }
    }
}