
#include "manual_herding_rules.h"
#include "shepherding.h"
#include "better_herding.h"
#include "multi_herding.h"
#include "deform_herding.h"

bool CManualHerdingFlock::initialize(list< list<string> >& tokens, flock_raw_data& data)
{
	list< list<string> > tokens_clone = tokens;
	bool r = CSimpleHerdingFlock::initialize(tokens_clone, data);
	if (r == false) return false; //failed

	delete getForceRule();
	delete getBehaviorRule();
	this->setForceRule(new ManualForceRule());
	this->setBehaviorRule(new ManualHerdingBehaviorRule());

	//------------------------------> check if auto piloting is provided
	string auto_pilot_type;
	for (auto i = tokens.begin(); i != tokens.end(); i++){
		//
		string label = tolower(i->front());
		cout << "label=" << label << endl;
		i->pop_front();
		//
		if (label == "auto_pilot")
		{
			if (i->empty()){ cerr << "! ERROR: manual shepher: auto_pilot has no value" << endl; return false; }
			auto_pilot_type = tolower(i->front());
			break;
		}
	}

	cout << "auto_pilot_type=" << auto_pilot_type << endl;
	if (auto_pilot_type.empty() == false)
	{
		m_auto_pilot = NULL;
		//create this auto pilot shepherd as the proxy
		if (auto_pilot_type == "simple"){
			m_auto_pilot = new CSimpleHerdingFlock(getEnvironment(), data.geo);
		}
		else if (auto_pilot_type == "better"){
			m_auto_pilot = new CBetterHerdingFlock(getEnvironment(), data.geo);
		}
		else if (auto_pilot_type == "multiple"){
			m_auto_pilot = new CMultiHerdingFlock(getEnvironment(), data.geo);
		}
		else if (auto_pilot_type == "deform"){
			m_auto_pilot = new CDeformHerdingFlock(getEnvironment(), data.geo);
		}

		if (m_auto_pilot != NULL)
		{
			data.size = 0; //make sure that no extra flock states are created
			bool r=m_auto_pilot->initialize(tokens, data);
			if (r == false){
				delete m_auto_pilot;
				m_auto_pilot = NULL;
				cerr << "! Error: manual shepher: auto_pilot initialization error. Ignored." << endl;
			}
		}
	}//auto_pilot_type.empty() == false


	sh_Draw * renderer = getSimulator()->getRenderer();
	if (renderer != NULL){
		if (renderer->isCompatible(this)){
			renderer->addDrawObj(this);
		}
	}

	return true;

}


void CManualHerdingFlock::draw()
{
	glLineWidth(2);
	for (auto s : this->getStates()) 
	{
		if (s->getID() == 0) //the first shepherd
		{
			if (this->getAutoPilotMode(s) == false)
				glColor3d(1, 1, 0);
			else
				glColor3d(0.5, 0.5, 0);
		}
		else
		{
			if (this->getAutoPilotMode(s) == false)
				glColor3d(0, 1, 1);
			else
				glColor3d(0, 0.5, 0.5);
		}

			glPushMatrix();
			glTranslated(s->getPos()[0],0.4f,s->getPos()[1]);
			drawCircle(this->getGeometry().getRadius()*2.5f,PI2);
			glPopMatrix();
	}
	glLineWidth(1);
}


Vector2d ManualForceRule::getForce(CFlockState & s)
{
    CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
	CManualHerdingFlock* shepherd_type = dynamic_cast<CManualHerdingFlock*>(s.getType());
	assert(shepherd_type);

	//check if it is auto-piloting
	if (shepherd_type->getAutoPilotMode(&s) == true)
	{
		return shepherd_type->getAutoPilot()->getForceRule()->getForce(s);
	}

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

void ManualHerdingBehaviorRule::applyRule(CFlockState& s)
{
    CHerdingFlockState& shepherd = (CHerdingFlockState&)s;
	CManualHerdingFlock* shepherd_type = dynamic_cast<CManualHerdingFlock*>(s.getType());
	assert(shepherd_type);
	shepherd_type->setAutoPilotMode(&s, false);

	//check if the goal is reached.
	if(shepherd.getID()==0) reachGoal(shepherd);

#ifdef _WIN32
	if (getSI()->getXBoxController() != NULL)
	{
		CXBOXController * controller = getSI()->getXBoxController();
		assert(controller);
		auto& gamepad = controller->GetState().Gamepad;
		float dx = 0, dy = 0;

		if (shepherd.getID() == 0)
		{
			if (std::abs(gamepad.sThumbRX) / 32768.0 > 0.2 || std::abs(gamepad.sThumbRY) / 32768.0 > 0.2)
			{
				dx =  gamepad.sThumbRX / 32768.0;
				dy = -gamepad.sThumbRY / 32768.0;
				shepherd.target = shepherd.getPos() + Vector2d(dx, dy);
			}
			else {
				//take over this shepherd
				perform_auto_pilot(shepherd);
			}
		}
		else //other shepherds
		{
			if (std::abs(gamepad.sThumbLX) / 32768.0 > 0.2 || std::abs(gamepad.sThumbLY) / 32768.0 > 0.2)
			{
				dx = gamepad.sThumbLX / 32768.0;
				dy = -gamepad.sThumbLY / 32768.0;
				shepherd.target = shepherd.getPos() + Vector2d(dx, dy);
			}
			else {
				//take over this shepherd
				perform_auto_pilot(shepherd);
			}
		}//end of (shepherd.getID() == 0)

		return; //done

	}//end of xbox controller

#endif //_WIN32

	//if no xbox controller user mouse
	if (shepherd.getID() == 0)
	{
		shepherd.target = shepherding_gui::getMousePosition();
	}
	else {
		//take over this shepherd
		perform_auto_pilot(shepherd);
	}//end if 
}


void ManualHerdingBehaviorRule::perform_auto_pilot(CHerdingFlockState&  s)
{
	CManualHerdingFlock* shepherd_type = dynamic_cast<CManualHerdingFlock*>(s.getType());
	if (shepherd_type->getAutoPilot() == NULL) return;
	shepherd_type->setAutoPilotMode(&s, true);

	//get all other shepherds
	//back up targets if the shepherd is controlled manually.
	map<CHerdingFlockState *, Point2d> backup_targets;
	for (auto a : getEnvironment()->getFlockStates()){
		if (a == &s) continue;
		CHerdingFlockState* shepherd = dynamic_cast<CHerdingFlockState*>(a);
		if (shepherd == NULL) continue;
		if (shepherd_type->getAutoPilotMode(shepherd) == false)
			backup_targets[shepherd] = shepherd->target;
	}//end

	//perform autopilot
	shepherd_type->getAutoPilot()->getBehaviorRule()->applyRule(s);
	
	//restore backup
	for (auto bk : backup_targets)
		bk.first->target = bk.second;
}




