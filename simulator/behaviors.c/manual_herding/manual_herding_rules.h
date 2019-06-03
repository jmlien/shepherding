
#pragma once
#include "simple_herding.h"
#include "simple_herding_rules.h"
#include "shepherding_gui.h"


class CManualHerdingFlock : public CSimpleHerdingFlock, gl_Draw
{
public:
	CManualHerdingFlock(CEnvironment * env, const string& name) :CSimpleHerdingFlock(env, name){ m_auto_pilot = NULL; }
	bool initialize(list< list<string> >& tokens, flock_raw_data& data);

	CHerdingFlock * getAutoPilot() { return m_auto_pilot; }
	void setAutoPilotMode(CFlockState* s, bool flag) { m_auto_pilot_modes[s] = flag; }
	bool getAutoPilotMode(CFlockState* s) { return m_auto_pilot_modes[s]; }

	void draw();

private:
	CHerdingFlock *  m_auto_pilot;
	map<CFlockState*, bool> m_auto_pilot_modes;
};


class ManualForceRule : public CBasicForceRule
{
public:
	virtual Vector2d getForce(CFlockState & s);
};

class ManualHerdingBehaviorRule : public CSimpleHerdingBehaviorRule
{
public:
	virtual void applyRule(CFlockState& s);

protected:
	void perform_auto_pilot(CHerdingFlockState& s);
};




