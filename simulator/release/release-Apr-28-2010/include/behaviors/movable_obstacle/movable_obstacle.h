#pragma once
#include "shepherding_base.h"
#include "./intersection.h"
#include "../herding_flock.h"

class MovableObstacleForceRule : public CBasicForceRule
{
    public:
    virtual Vector2d getForce(CFlockState & s)
    {
        std::vector<CHerdingFlockState*> shepherds = MovableObstacleForceRule::findShepherds(s);
        const int size = shepherds.size();
        Vector2d force = s.getVelocity()*(-s.getType()->getMass());     // cancel out obstacles velocity (simulate 100% kinetic friction)
        float rotation = s.getRot();
        for(int i = 0; i < size; i++)
        {
            const CHerdingFlockState& shepherd = *shepherds[i];
            if(shepherd.getVelocity()*(s.getPos() - shepherd.getPos()) > 0)
            {
                std::vector<Point2d> collision = intersection(s, shepherd);
                if(collision.size() > 0)
                {
                    Vector2d shepherd_velocity = shepherd.getVelocity();
                    Vector2d collision_vector = s.getPos() - collision.front();
                    float collision_velocity = shepherd_velocity.norm();
                    float collision_radius = collision_vector.norm();
                    float length_product = collision_velocity*collision_radius;
                    float torque_angle = length_product != 0.0 ? acos((shepherd_velocity*collision_vector)/length_product) : 0.0;
                    float torque_radius = sin(torque_angle)*collision_radius;
                    float rotational_velocity;
                    if(shepherd_velocity[0]*collision_vector[1] - shepherd_velocity[1]*collision_vector[0] > 0.0)
                    {
                        rotational_velocity = -torque_radius*collision_velocity;
                    }
                    else
                    {
                        rotational_velocity = torque_radius*collision_velocity;
                    }
                    
                    force = force + shepherd_velocity*s.getType()->getMass();
                    rotation += rotational_velocity;
                }
            }
        }
        s.setRot(rotation);
        return force;
    }
    
    private:
    static std::vector<CHerdingFlockState*> findShepherds(CFlockState & s)
    {
        //list<CFlockState*> visible_agents = s.getVisibleAgent();
        list<CFlockState*> visible_agents = getEnvironment()->getFlockStates();
        std::vector<CHerdingFlockState*> shepherds;
        shepherds.reserve(visible_agents.size()/2);
        for(list<CFlockState*>::iterator i = visible_agents.begin(), end = visible_agents.end(); i != end; i++)
        {
            CHerdingFlockState* shepherd = dynamic_cast<CHerdingFlockState*>(*i);
            if(shepherd != 0)
            {
                shepherds.push_back(shepherd);
            }
        }
        return shepherds;
    }

};


class MovableObstacleBehaviorRule : public CBehaviorRule
{
    public:
    virtual void applyRule(CFlockState& s)
    {
        // do nothing--movable obstacles have no autonomous behavior
    }
};



