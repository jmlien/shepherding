#ifndef ZEROFORCERULE_H
#define ZEROFORCERULE_H

#include "sh_ForceRules.h"

class ZeroForceRule : public CBasicForceRule
{
    
    public:
        ZeroForceRule()  {}
        virtual Vector2d getForce(CFlockState & s)
        {
            return Vector2d(0.0, 0.0);
        }

};

#endif // ZEROFORCERULE_H

