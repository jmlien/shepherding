#ifndef _SH_BEHAVIOR_RULES_H_
#define _SH_BEHAVIOR_RULES_H_

class CFlockState;

///////////////////////////////////////////////////////////////////////////////
class CBehaviorRule
{
public:
	//CBehaviorRule(){/*do not thing*/}
	//CBehaviorRule(const CBehaviorRule& other){/*do not thing*/}
    virtual void applyRule( CFlockState& s ) =0;
};

class CBasicBehaviorRule: public CBehaviorRule
{
public:
    CBasicBehaviorRule(){/*do not thing*/}
    //CBasicBehaviorRule(const CBasicBehaviorRule& other){/*do not thing*/}
    virtual void applyRule( CFlockState& s ){ /*do not thing*/ };
};

#endif //_SH_BEHAVIOR_RULES_H_


