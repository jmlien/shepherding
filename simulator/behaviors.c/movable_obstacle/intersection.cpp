

#include "./intersection.h"


// formats the flock state's attributes, so that they can be passed to RAPID_collide
struct CollisionInfo
{
    CRobot2D* robot;
    RAPID_model* model;
    double rotation[3][3];
    double translation[3];
};

// fills in the collision info struct for a given flock state
CollisionInfo extractCollisionInfo(const CFlockState& flock_state)
{
    CollisionInfo collision_info;
    collision_info.robot = &(flock_state.getType()->getGeometry());
    collision_info.model = &(collision_info.robot->getCDModel());
    const float* rotation_vector = flock_state.getRotM();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            collision_info.rotation[i][j] = rotation_vector[i*3 + j];
        }
    }
    const Point2d& position = flock_state.getPos();
    collision_info.translation[0] = position[0];
    collision_info.translation[1] = 0.0;
    collision_info.translation[2] = position[1];
    return collision_info;
}

// fills the ouput parameter "triangle" with the transformed vertices from the triangle at index "triangle_id"
void extractTriangle(const CollisionInfo& collision_info, int triangle_id, Point3d triangle[3])
{
    const IModel::Tri& vertices = collision_info.robot->getTri()[triangle_id];
    Matrix3x3 rotation;
    rotation.set(collision_info.rotation);
    Vector3d translation(collision_info.translation[0], collision_info.translation[1], collision_info.translation[2]);
    for(int i = 0; i < 3; i++)
    {
        Point3d vertex = collision_info.robot->getGeo()[vertices[i]];
        triangle[i] = Point3d(rotation*Vector3d(vertex[0], vertex[1], vertex[2]) + translation);
    }
}

// used to average two intersecting triangles (to approximate point of intersection)
Point2d averageTriangles(const Point3d a[3], const Point3d b[3])
{
    Vector2d average_a(0.0, 0.0);
    Vector2d average_b(0.0, 0.0);
    for(int i = 0; i < 3; i++)
    {
        average_a[0] += a[i][0];
        average_a[1] += a[i][2];
        average_b[0] += b[i][0];
        average_b[1] += b[i][2];
    }
    average_a = average_a*(1.0/3.0);
    average_b = average_b*(1.0/3.0);
    return Point2d(0.5*(average_a[0] + average_b[0]), 0.5*(average_a[1] + average_b[1]));
}

// computes the intersection of two flock states using RAPID_collide
std::vector<Point2d> intersections(const CFlockState& a, const CFlockState& b, int rapid_flags)
{
    CollisionInfo collision_info_a = extractCollisionInfo(a), collision_info_b = extractCollisionInfo(b);
    RAPIDres result;
    RAPID_Collide(result, collision_info_a.rotation, collision_info_a.translation, collision_info_a.model,
        collision_info_b.rotation, collision_info_b.translation, collision_info_b.model, rapid_flags);
    
    const int size = result.RAPID_num_contacts;
    std::vector<Point2d> intersections;
    intersections.reserve(size);
    for(int i = 0; i < size; i++)
    {
        Point3d triangle_a[3], triangle_b[3];
        extractTriangle(collision_info_a, result.RAPID_contact[i].id1, triangle_a);
        extractTriangle(collision_info_b, result.RAPID_contact[i].id2, triangle_b);
        intersections.push_back(averageTriangles(triangle_a, triangle_b));
    }
    return intersections;
}

// finds a point of intersection between two flock states
std::vector<Point2d> intersection(const CFlockState& a, const CFlockState& b)
{
    return intersections(a, b, RAPID_FIRST_CONTACT);
}

// finds all points of intersection between two flock states
std::vector<Point2d> intersections(const CFlockState& a, const CFlockState& b)
{
    return intersections(a, b, RAPID_ALL_CONTACTS);
}

Vector2d parallelComponent(const Vector2d& direction, const Vector2d& collision_vector)
{
    if(direction*collision_vector > 0.0)
    {
        return collision_vector*((direction*collision_vector)/collision_vector.normsqr());
    }
    return Vector2d(0.0, 0.0);
}


