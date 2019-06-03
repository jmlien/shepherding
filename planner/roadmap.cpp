#include "mp.h"

Roadmap_Node Roadmap_Node::InvalidData() {
	Roadmap_Node tmp;
	tmp.m_id = -1;
	cerr << "! Warning: Roadmap_Node InvalidData() is evoked." << endl;
	return tmp;
}

bool Roadmap_Node::operator==(const Roadmap_Node& n) const {
	return n.m_id == m_id;
}

MetaRoadmap_Node MetaRoadmap_Node::InvalidData() {
	MetaRoadmap_Node tmp;
	tmp.m_id = -1;
	cerr << "! Warning: MetaRoadmap_Node InvalidData() is evoked." << endl;
	return tmp;
}

bool MetaRoadmap_Node::operator==(const MetaRoadmap_Node& n) const {
	return n.m_id == m_id;
}

