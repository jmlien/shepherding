
#pragma once
#include "simple_herding_rules.h"
#include "./shepherding_gui.h"

class ManualForceRule : public CBasicForceRule
{
    public:
    virtual Vector2d getForce(CFlockState & s)
    {
        CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
        Vector2d to_target = shepherd.target - shepherd.getPos();
        float distance = to_target.norm();
        float v = shepherd.getVelocity().norm();
        if(distance == 0.0)
        {
            return (v != 0.0 ? shepherd.getVelocity()*(-this->getMaxForce()/v) : Vector2d(0.0, 0.0));
        }
        else if(v == 0.0)
        {
            return to_target*(this->getMaxForce()/distance);
        }
        else
        {
            float target_v = (to_target*shepherd.getVelocity())/distance;
            Vector2d target_velocity = to_target*(target_v/distance);
            Vector2d correction_velocity = target_velocity - shepherd.getVelocity();
            float correction_v = correction_velocity.norm();
            
            float abs_target_v = fabs(target_v);
            float total_v = abs_target_v + correction_v;
            float target_ratio = abs_target_v/total_v;
            float correction_ratio = 1.0 - target_ratio;
            
            Vector2d acceleration(0.0, 0.0);
            float max_acceleration = this->getMaxForce()/shepherd.getType()->getMass();
            if(correction_ratio > 0.01)
            {
                acceleration = correction_velocity*(correction_ratio*max_acceleration/correction_v);
            }
            if(target_ratio > 0.01)
            {
                float deceleration_threshold = abs_target_v*target_v/(2.0*target_ratio*max_acceleration);
                if(distance > deceleration_threshold)
                {
                    acceleration = acceleration + to_target*(target_ratio*max_acceleration/distance);
                }
                else
                {
                    acceleration = acceleration - to_target*(target_ratio*max_acceleration/distance);
                }
            }
            float length = acceleration.norm();
            return (length != 0.0 ? acceleration*(this->getMaxForce()/length) : Vector2d(0.0, 0.0));
        }
    }
};

class ManualHerdingBehaviorRule: public CSimpleHerdingBehaviorRule 
{
    public:
    virtual void applyRule(CFlockState& s)
    {
        CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
        shepherd.target = shepherding_gui::getMousePosition();
    }
};




