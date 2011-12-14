/**********************************************************************\
* Dateiname: planner.cpp
* Autor : Mario Grotschar
* Projekt : project3_astar
* Copyright (C) <<COPYRIGHT>>
*
*
* Datum: Autor: Grund der Aenderung:
* 7.12.2011 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <ctype.h>
#include <string>
#include <string.h>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <iostream>
#include <math.h>

#include "stlastar.h"
#include "agent.h"


using namespace std;

// --------------
// NodeSearch Astar
// --------------



bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when the mIDs match
	if( (room->mID == rhs.room->mID))
	{
		return true;
	}
	else
	{
		return false;
	}
}

Room * MapSearchNode::getRoom()
{
	return room;
}

void MapSearchNode::PrintNodeInfo()
{
	//cout << "Node position : (" << room->mColumn << ", " << room->mRow << ")" << endl;
	room->mFlagAstar = true;
}

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	float xd = fabs(float(((float)room->mColumn - (float)nodeGoal.room->mColumn)));
	float yd = fabs(float(((float)room->mRow - (float)nodeGoal.room->mRow)));

	return xd + yd;
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( (room->mID == nodeGoal.room->mID) )
	{
		return true;
	}

	return false;
}


// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	//int parent_x = -1; 
	//int parent_y = -1; 
	Room dummy_room (-1, -1, -1);
	Room *parent_room = &dummy_room;

	if( parent_node )
	{
		parent_room = parent_node->room;
		//parent_x = parent_node->x;
		//parent_y = parent_node->y;
	}
	

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards
	
	// Check Westnode
	if (room->hasWest() && (parent_room != NULL) && room->isFree())
	{
		// We dont want to go back.
		if ((room->mWest->mCost < 9) && !(parent_room->mID == room->mWest->mID))
		{
			NewNode = MapSearchNode (room->mWest);
			astarsearch->AddSuccessor( NewNode );
		}
	}

	// Check Southnode
	if (room->hasSouth()&& (parent_room != NULL) && room->isFree())
	{
		// We dont want to go back.
		if ((room->mSouth->mCost < 9) && !(parent_room->mID == room->mSouth->mID))
		{
			NewNode = MapSearchNode (room->mSouth);
			astarsearch->AddSuccessor( NewNode );
		}
	}

	// Check Eastnode
	if (room->hasEast()&& (parent_room != NULL) && room->isFree())
	{
		// We dont want to go back.
		if ((room->mEast->mCost < 9) && !(parent_room->mID == room->mEast->mID))
		{
			NewNode = MapSearchNode (room->mEast);
			astarsearch->AddSuccessor( NewNode );
		}
	}

	// Check Northnode
	if (room->hasNorth() && (parent_room != NULL) && room->isFree())
	{
		// We dont want to go back.
		if ((room->mNorth->mCost < 9) && !(parent_room->mID == room->mNorth->mID))
		{
			NewNode = MapSearchNode (room->mNorth);
			astarsearch->AddSuccessor( NewNode );
		}
	}

/*
		
	if( (GetMap( x, y+1 ) < 9) 
		&& !((parent_x == x) && (parent_y == y+1))
		)
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	
	*/

	return true;
}


// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) successor.room->mCost;

}