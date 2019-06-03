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
class CObsState{
public:

    void  setPos(const Point2d& pos){ m_Position=pos; }
    const Point2d& getPos() const { return m_Position; }

    void  setRot(float y){ float ry=(y/180)*PI; rotateY(m_R,ry); }
    float * getRot() const { return (float*)m_R; }

    void setColor( float r, float g, float b ){ 
        m_Color[0]=r; m_Color[1]=g; m_Color[2]=b;
    }
    const Point3d& getColor() { return m_Color; }

    void setTexture( const string& file ){ m_TextureFile=file; }
    const string& getTexture() const { return m_TextureFile; } 

    //////////////////////////////////////////////////////////////////////////
    // Type, ID

    CObs * getType() const { return m_Type; }
    int getID(){ return m_ID; }

private:

    //id
    static int TOTAL_STATE_SIZE;
    int m_ID;

    friend class CObs;
    CObsState(CObs * type);

    Point2d m_Position;
    float m_R[3][3];
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
    ~CObs();

    ///////////////////////////////////////////////////////////////////////////
    void Configure(const CObsState& s){ 
        const Point2d & pos=s.getPos();
        m_Obs.setPos(pos[0],0,pos[1]);
        m_Obs.setRot(s.getRot());
    }
    
    ///////////////////////////////////////////////////////////////////////////
    //Access
    CObsState& getState(int i){ return *m_State[i]; }
    int getStateSize() const { return m_State.size(); }
    vector<CObsState*>& getStates(){ return m_State; }
    CObs2D& getGeometry() { return m_Obs; }
    IModel * getRawModel() { return m_Model; }
    int getID(){ return m_ID; }

    ///////////////////////////////////////////////////////////////////////////
    //Add/Remvoe
    CObsState& addState();

protected:
    
    void initialize(IModel * model, int size);
    
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


