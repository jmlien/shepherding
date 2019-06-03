#ifndef MANUAL_SHEPHERD_RULES_H
#define MANUAL_SHEPHERD_RULES_H

#include "simple_herding_rules.h"

class ManualTargetForceRule : public CBasicForceRule
{
    
    public:
        ManualTargetForceRule(int maxNumShepherds);
        virtual Vector2d getForce(CFlockState & s);

        virtual Vector2d getManualForce(CFlockState & s, int idx);
        virtual Vector2d getAutonomousForce(CFlockState & s);


        void setOldPosition(int i,  Point2d oldPosition)    {  m_oldPositions.at(i) = oldPosition;   }
        void setGoalPosition(int i, Point2d goalPosition)   {  m_goalPositions.at(i) = goalPosition; }

        Point2d getOldPosition(int i)                 { return m_oldPositions.at(i);  }
        Point2d getGoalPosition(int i)                { return m_goalPositions.at(i); }
        
        void addAutonomousForceRule( CBasicForceRule * autonomousForceRule ) {
            m_autonomousForceRule = autonomousForceRule;
        }


    private:
        int m_maxNumShepherds;
        vector<Point2d> m_goalPositions;
        vector<Point2d> m_oldPositions;
        CBasicForceRule * m_autonomousForceRule;
};

class CManualHerdingBehaviorRule: public CSimpleHerdingBehaviorRule 
{
public:

    virtual void applyRule( CFlockState& s );
};



#endif // ManualTargetForceRule.h


