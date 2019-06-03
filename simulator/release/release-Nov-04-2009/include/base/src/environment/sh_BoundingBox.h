// BoundingBox.h: interface for the CBoundingBox class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_BOUNDINGBOX_H__634C3257_4A20_408D_90F4_3E632CFA83FF__INCLUDED_)
#define AFX_BOUNDINGBOX_H__634C3257_4A20_408D_90F4_3E632CFA83FF__INCLUDED_

#include <Vector.h>
#include <Point.h>
using namespace mathtool;

#include <string>

class CBoundingBox  
{
public:
    //////////////////////////////////////////////////////////////////////
    //Constructors/Destructor
    //////////////////////////////////////////////////////////////////////
    CBoundingBox();
    virtual ~CBoundingBox();

    //////////////////////////////////////////////////////////////////////
    //Access
    const float * getBBXValue() const{ return m_BBX; }
    void setBBXValue(const float * bbx){ memcpy(m_BBX,bbx,sizeof(float)*6); }
    void setTexture(const std::string& texture){ m_Texture=texture; }
    const std::string& getTexture()const{ return m_Texture; }

    //////////////////////////////////////////////////////////////////////
    //Collision info
    bool createCSPace(float Radius);
    //return true means collision found and information is put in corrosponding variables.
    bool getCollisionInfo( const Point2d & oldPos, const Point2d & newPos,
                           Vector2d & Normal/*out*/, float & dCP/*out, collision point*/ );
    bool isCollision( const Point2d & oldPos );
    float getClearance( const Point2d & pos, Point2d & cd_pt );
    Point2d getRandomPoint(); //randomly generate point size cbbx
	void Push(Point2d & Pos);

//////////////////////////////////////////////////////////////////////
//Private & Protected
protected:
    //Check collision
    inline void getCollisionInfo
    (float dNew, float dOld, float dPlane, 
     Vector2d& oldNormal, Vector2d& newNormal,float& dCP) const;
    //Check collision in Left Plane
    void getCollisionInfo_Left
    (const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const;
    //Check collision in Right Plane
    void getCollisionInfo_Right
    (const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const;
    //Check collision in Front Plane
    void getCollisionInfo_Front
    (const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const;
    //Check collision in Back Plane
    void getCollisionInfo_Back
    (const Point2d& oldPos,const Point2d& newPos,Vector2d& Normal,float& dCP)const;

    //Check if give point is in bbx
    bool isInBBX( const Point2d & Pos ) const { 
        return 
        (m_CBBX[0]<=Pos[0]&&m_CBBX[1]>=Pos[0]&&m_CBBX[4]<=Pos[1]&&m_CBBX[5]>=Pos[1]);
    }

    void buildModel();

private:
    float m_BBX[6];
    float m_CBBX[6]; //Cspace BBX
    std::string m_Texture;
};

#endif // !defined(AFX_BOUNDINGBOX_H__634C3257_4A20_408D_90F4_3E632CFA83FF__INCLUDED_)

