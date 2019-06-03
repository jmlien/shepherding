

#ifndef BALANCED_FORCE_HPP
#define BALANCED_FORCE_HPP


#include <cmath>
#include "simple_herding_rules.h"
#include "sh_ForceRules.h"


namespace Balanced {


class FORCE : public CSimpleHerdingForceRule
{
    public:
    virtual Vector2d getForce(CFlockState& s)
    {
        CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
        
        Vector2d target_vector = shepherd.target - shepherd.getPos();
        const float target_distance = target_vector.norm();
        
        Vector2d current_velocity = shepherd.getVelocity();
        Vector2d target_velocity = target_vector.normalize()*((current_velocity*target_vector)/target_distance);
        Vector2d correction_velocity = target_velocity - current_velocity;
        
        const float target_speed = target_velocity.norm();
        const float current_speed = current_velocity.norm();
        const float correction_speed = correction_velocity.norm();
        const float brake_speed = current_speed + correction_speed;
        const float speed_ratio = current_speed/brake_speed;
        const float correction_ratio = 1.0f - speed_ratio;
        
        const float max_force = this->CBasicForceRule::getMaxForce();
        //float max_force = 100;
        const float acceleration = max_force/shepherd.getType()->getMass();
        const float distance_limit = (current_speed*current_speed)/(2.0f*speed_ratio*acceleration)*(target_speed/current_speed);
        
        // if getting close to target, starting using the brakes
        if(target_distance <= distance_limit)
        {
            return (correction_velocity.normalize()*correction_ratio -
                current_velocity.normalize()*speed_ratio).normalize()*max_force;
        }
        // else, continue approaching target at max acceleration
        else
        {
            const float total_speed = target_speed + correction_speed;
            return (correction_velocity.normalize()*(correction_speed/total_speed) +
                target_vector.normalize()*(target_speed/total_speed)).normalize()*max_force;
        }
    }
};


} // end Balanced


#endif // BALANCED_FORCE_HPP


