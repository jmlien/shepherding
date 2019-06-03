// AniModel.h: interface for the CAniModel class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(_ANI_MODEL_H_)
#define _ANI_MODEL_H_

#include "sh_PolyhedronModel.h"
#include <vector>

//a class construct a polyhedron body
class CAniModel : public CPolyhedronModel
{
public:

    //////////////////////////////////////////////////////////////////////
    // Constructor/Destructor
    CAniModel();
    virtual ~CAniModel();

    virtual void doDraw(Vector3d & color);

    //methods for building GL models
    virtual bool buildGLModel(IModel& model);

    //access
    virtual bool addGLModel(IModel& model);
    int getModelSize() const { return m_GLIDs.size(); }
    void Update(int index){ m_Index=index; }
    
    void setStepSize(float s) { m_StepSize=s; }
    float getStepSize() const { return m_StepSize; }

//////////////////////////////////////////////////////////////////////
// Protected and Private
protected:

    std::vector<int> m_GLIDs;
    int m_Index;
    float m_StepSize;
};

#endif // !defined(_ANI_MODEL_H_)

