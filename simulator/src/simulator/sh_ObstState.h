#ifndef _SH_OBST_STATE_H_
#define _SH_OBST_STATE_H_

//////////////////////////////////////////////////////////////////////////
#include <Point.h>
#include <Basic.h>
#include "sh_Obs2D.h"
//////////////////////////////////////////////////////////////////////////
#include <list>
using namespace std;
//////////////////////////////////////////////////////////////////////////
#include "model/ModelFactory.h"
//////////////////////////////////////////////////////////////////////////
//State for each instance of obstacle
class CObs;
class CObsState
{
public:

    virtual ~CObsState(){} //destructor

    virtual void  setPos(const Point2d& pos){ m_Position=pos; }
    virtual const Point2d& getPos() const { return m_Position; }

    virtual void  setRot(float y){ m_radian=(y/180)*PI; rotateY(m_R,m_radian); }
    virtual float * getRot() const { return (float*)m_R; }
    virtual float getRotRadian() const { return m_radian; }

    virtual void setColor( float r, float g, float b ){
        m_Color[0]=r; m_Color[1]=g; m_Color[2]=b;
    }
    virtual const Point3d& getColor() { return m_Color; }

    virtual void setTexture( const string& file ){ m_TextureFile=file; }
    virtual const string& getTexture() const { return m_TextureFile; }

    //////////////////////////////////////////////////////////////////////////
    // Type, ID
    virtual CObs * getType() const { return m_Type; }
    virtual int getID(){ return m_ID; }

private:

    //id
    static int TOTAL_STATE_SIZE;
    int m_ID;

    friend class CObs;
    CObsState(CObs * type);

    Point2d m_Position;
    float m_R[3][3];
    float m_radian;
    Point3d m_Color;
    CObs * m_Type;
    int m_Texture;
    string m_TextureFile;
};

//////////////////////////////////////////////////////////////////////////
//State for each type of obstacle
class CObs{
public:
    //Constructor
    CObs(const string& filename, int size);
    CObs(IModel * model, int size);
    virtual ~CObs();

    ///////////////////////////////////////////////////////////////////////////
    virtual void Configure(const CObsState& s){
        const Point2d & pos=s.getPos();
        m_Obs.setPos(pos[0],0,pos[1]);
        float * r=s.getRot();
        m_Obs.setRot(r);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    //Access
    virtual CObsState& getState(int i){ return *m_State[i]; }
    virtual int getStateSize() const { return m_State.size(); }
    virtual vector<CObsState*>& getStates(){ return m_State; }
    virtual CObs2D& getGeometry() { return m_Obs; }
    virtual IModel * getRawModel() { return m_Model; }
    virtual int getID(){ return m_ID; }

    ///////////////////////////////////////////////////////////////////////////
    //Add/Remvoe
    virtual CObsState& addState();

protected:
    
    virtual void initialize(IModel * model, int size);
    
private:

    //id
    static int TOTAL_OBS_SIZE;
    int m_ID;

    //Data
    CObs2D m_Obs;           //the geo
    IModel * m_Model;       //the abstract model

    //instances
    vector<CObsState*> m_State; //< Cfgs of obs instances
};

#endif


