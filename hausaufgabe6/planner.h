/**********************************************************************\
* Dateiname: planner.h
* Autor : Mario Grotschar
* Projekt : project3_astar
* Copyright (C) <<COPYRIGHT>>
*
*
* Datum: Autor: Grund der Aenderung:
* 8.12.2011 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/


#ifndef _INCL_PLANNER
#define _INCL_PLANNER

#include <iostream>

#include "maze.h"
#include "stlastar.h"
// Definitions



using namespace std;

class MapSearchNode
{
public:
	//unsigned int x;	 // the (x,y) positions of the node
	//unsigned int y;	
	Room * room;
	
	MapSearchNode() { room = NULL; }
	MapSearchNode( Room * setRoom ) { room=setRoom; }

	Room * getRoom();

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 


};

#endif