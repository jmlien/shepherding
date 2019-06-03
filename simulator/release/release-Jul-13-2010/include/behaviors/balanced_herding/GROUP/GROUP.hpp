

#ifndef BALANACED_GROUP_GROUP_HPP
#define BALANACED_GROUP_GROUP_HPP


#include "../CIRCLE/CIRCLE.hpp"


namespace Balanced {


struct GROUP : public CIRCLE
{
    std::vector<CFlockState*> sheep;
    
    GROUP(const CIRCLE& circle, const std::vector<CFlockState*>& sheep) :
        CIRCLE(circle),
        sheep(sheep)
    {
        // do nothing
    }
    
    static GROUP Construct(const std::vector<CFlockState*>& sheep)
    {
        return GROUP(CIRCLE::Enclosing(sheep), sheep);
    }   
};


} // end Balanced


#endif // BALANACED_GROUP_GROUP_HPP


