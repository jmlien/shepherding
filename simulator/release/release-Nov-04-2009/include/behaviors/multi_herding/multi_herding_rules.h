#ifndef MULTI_SHEPHERD_RULES_H
#define MULTI_SHEPHERD_RULES_H

#include "better_herding_rules.h"
#include "pickSteeringPos.h"
#include "shepherdFormation.h"
#include "distributeShepherds.h"


class CMultiHerdingForceRule : public CBasicForceRule
{
public:
	CMultiHerdingForceRule(){TargetAttract=1;}
    virtual Vector2d getForce(CFlockState & s);
	int TargetAttract;
};


class CMultiHerdingBehaviorRule: public CBetterHerdingBehaviorRule
{
public:

	CMultiHerdingBehaviorRule();
    virtual void applyRule( CFlockState& s );

protected:

	simpleMinimization * chooseTarget;
	DistributeShepherdsGroupSizeClose * distribution;
	circleFormation * formation;

	//compute groups, then compute milestone, then find target
    bool findTarget(CHerdingFlockState& s, Point2d& target);

	void checkTargetAndGroupCenter(CHerdingFlockState& s, Point2d& C_f);

	//create groups in the flockgroup format
	void createGroups(FSLIST& flocks, vector<FlockGroup>& groups);

	// classify agents into flock and shepherds
	void classifyAgents(CHerdingFlockState& s, FSLIST& va, FSLIST& shepherds, FSLIST& flocks);

	//avg group vec
	Vector2d groupVec(FlockGroup& g);

	// target is too far, use the first milestone
	// on the path to target as the target.
	void goToTarget(CHerdingFlockState& s);

	// check if s is too far from a group
	// R,O are radius and center of the group under consideration
	bool isFarFromTarget(CHerdingFlockState& s);

	// compute milestones for ALL groups
	void findMilestones(CHerdingFlockState& s, vector<FlockGroup>& groups,float r);
};


#endif // MULTI_SHEPHERD_RULES_H


