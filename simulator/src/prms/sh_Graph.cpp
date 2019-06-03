#include "sh_Graph.h"

/////////////////////////////////////////////////////////////////////
//
// CNode
//
/////////////////////////////////////////////////////////////////////

CNode CNode::m_InvalidValue;

CNode::CNode()
{
    m_ID=-1;
    m_Clear=0;
}

CNode::~CNode()
{

}

bool CNode::operator==( const CNode & other ) const
{
    return (m_Pos[0]==other.m_Pos[0]&&m_Pos[1]==other.m_Pos[1]&&m_ID==other.m_ID);
}

ostream & operator<<( ostream & out, const CNode & node )
{
    return out;
}

istream & operator>>( istream &  in, CNode & node )
{
    return in;
}

/////////////////////////////////////////////////////////////////////
//
// CEdge
//
/////////////////////////////////////////////////////////////////////

CEdge CEdge::m_InvalidValue;

CEdge::CEdge()
{
    m_Weight=1;
}

CEdge::CEdge(float weight)
{
    m_Weight=(float)weight;
}

CEdge::~CEdge()
{

}

bool CEdge::operator==( const CEdge & other ) const
{
    return other.m_Weight==m_Weight;
}

ostream & operator<<( ostream & out, const CEdge & node )
{
    return out;
}

istream & operator>>( istream &  in, CEdge & node )
{
    return in;
}

