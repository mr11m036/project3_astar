/**********************************************************************\
* Dateiname: agent.cpp
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

#include "maze.h"


using namespace std;
// --------------
// Room
// --------------

Room::Room (int id, int row, int column):
mID (id),
mRow (row),
mColumn (column),
mGroupID (0),
mNorth (0),
mSouth (0),
mEast (0),
mWest (0),
mMaxRoom (0),
mMaxDepth (0),
mCost(1),
mFlag (false),
mFlagBFS (false),
mFlagIDS (false),
mFlagAstar(false),
occupiedRobot(NULL)

{
}

bool Room::bf_search(Group & visited, Room *startRoom , Room *exitRoom)
{
	//std::queue <Room *> *nodeQueue = new std::queue <Room *>;
	std::queue <Room *> nodeQueue;
	bool	exit_found = false;
	Room *	tempRoom = NULL;
	Room *	finalRoom = NULL;



	// Enqueue the root node.
	nodeQueue.push(startRoom);

	// Mark source. 
	//startRoom->mFlagBFS = true;
	
	startRoom->setParent(NULL);

	// Dequeue the nodes on the queue and anlyse them. As long as the queue is not empty.

	while (! nodeQueue.empty())
	{
		// Mark as visited.
		tempRoom = nodeQueue.front();
		visited.insert(tempRoom->mID);

		// Delete the anlysed Object from the Queue.
		nodeQueue.pop();

		// Check if we have reached the exit
		if (tempRoom->mID == exitRoom->mID)
		{
			tempRoom->mFlagBFS = true;
			finalRoom = tempRoom;
			exit_found = true;
			break;
		}

		// Analyse all its leaves
		if (tempRoom->mNorth && visited.find (tempRoom->mNorth->mID) == visited.end ())
		{
	 		tempRoom->mNorth->setParent(tempRoom);
			nodeQueue.push(tempRoom->mNorth);

		}
		
		if (tempRoom->mSouth && visited.find (tempRoom->mSouth->mID) == visited.end ())
		{
			tempRoom->mSouth->setParent(tempRoom);
			nodeQueue.push(tempRoom->mSouth);
		
 		}

		if (tempRoom->mWest && visited.find (tempRoom->mWest->mID) == visited.end ())
		{
			tempRoom->mWest->setParent(tempRoom);
			nodeQueue.push(tempRoom->mWest);
	
 		}

		if (tempRoom->mEast && visited.find (tempRoom->mEast->mID) == visited.end ())
		{
			tempRoom->mEast->setParent(tempRoom);
			nodeQueue.push(tempRoom->mEast);

 		}
	
	}
	

	// Set the flags for the shortest path beginning from the Exit to the Root.
	if (finalRoom && exit_found)
		while (!finalRoom->mParent == NULL)
		{
			finalRoom->mFlagBFS = true;
			finalRoom = finalRoom->mParent;
		}

	
	return exit_found;


}

bool Room::id_search (Group & visited, Room * exitRoom, int idDeph)
{
	bool exit_found = false;
	visited.insert(this->mID);
	
	if (idDeph >= 0)
	{
		if (this->mID == exitRoom->mID)
		{
			exit_found = true;
			this->mFlagIDS = true;
			return exit_found;
		}	
		
		if (this->mNorth && visited.find (this->mNorth->mID) == visited.end ())
		{
	 		this->mNorth->setParent(this);
			exit_found = this->mNorth->id_search(visited, exitRoom, idDeph--);
		}
		
		if (this->mSouth && visited.find (this->mSouth->mID) == visited.end ())
		{
			this->mSouth->setParent(this);
			exit_found = this->mSouth->id_search(visited,  exitRoom, idDeph--);
		
 		}

		if (this->mWest && visited.find (this->mWest->mID) == visited.end ())
		{
			this->mWest->setParent(this);
			exit_found = this->mWest->id_search(visited, exitRoom, idDeph--);	
	
 		}

		if (this->mEast && visited.find (this->mEast->mID) == visited.end ())
		{
			this->mEast->setParent(this);
			exit_found = this->mEast->id_search(visited,  exitRoom, idDeph--);	
		}
	}

	
	return exit_found;
}

bool Room::df_search(Group & visited, Room * exitRoom)
{
	bool	exit_found = false;
	visited.insert(mID);

	if (mNorth && visited.find (mNorth->mID) == visited.end ())
    {
     
      if (mNorth->df_search(visited, exitRoom)== true)
	  {
		  exit_found = true;
		  this->mFlag = true;
	  }
	  else
	  {
		  // do nothing
	  }
    }

	if (mSouth && visited.find (mSouth->mID) == visited.end ())
    {
      
      if (mSouth->df_search(visited, exitRoom) == true)
	  {
		  exit_found = true;
		  this->mFlag = true;
	  }
	  else
	  {
		  // do nothing
	  }
    }

	if (mWest && visited.find (mWest->mID) == visited.end ())
    {
      if (mWest->df_search(visited, exitRoom) == true)
	  {
		  exit_found = true;
		  this->mFlag = true;
	  }
	  else
	  {
		  // do nothing
	  }
    }

	if (mEast && visited.find (mEast->mID) == visited.end ())
    {
		if (mEast->df_search(visited, exitRoom) == true)
		{
			  exit_found = true;
			  this->mFlag = true;
		}
		else
		  {
		  // do nothing
		  }
    }

	// We found the exit room.
	if (!exit_found && (this->mID == exitRoom->mID))
		exit_found = true;

	return	exit_found;
}

void Room::visit (int depth, Group & visited, Room * caller)
{
  bool isDeadEnd = true;

  visited.insert (mID);

  if (mNorth && visited.find (mNorth->mID) == visited.end ())
    {
      isDeadEnd = false;
      mNorth->visit (depth + 1, visited, caller);
    }

  if (mSouth && visited.find (mSouth->mID) == visited.end ())
    {
      isDeadEnd = false;
      mSouth->visit (depth + 1, visited, caller);
    }

  if (mWest && visited.find (mWest->mID) == visited.end ())
    {
      isDeadEnd = false;
      mWest->visit (depth + 1, visited, caller);
    }

  if (mEast && visited.find (mEast->mID) == visited.end ())
    {
      isDeadEnd = false;
      mEast->visit (depth + 1, visited, caller);
    }

  if (isDeadEnd)
    {
      caller->deadEndFound (depth, this);
    }
}

void Room::deadEndFound (int depth, Room * room)
{
  if (depth > mMaxDepth)
    {
      mMaxDepth = depth;
      mMaxRoom = room;
    }
}

bool Room::getIdsFlag()
{
	return mFlagIDS;
}

Room * Room::getParent()
{
	return (mParent);
}

Room * Room::getMaxRoom ()
{
  return (mMaxRoom);
}

bool Room::hasNorth ()
{
  return (mNorth != NULL);
}

bool Room::hasSouth ()
{
  return (mSouth != NULL);
}

bool Room::hasEast ()
{
  return (mEast != NULL);
}

bool Room::hasWest ()
{
  return (mWest != NULL);
}

void Room::setNorth (Room * ptr)
{
  mNorth = ptr;
}

void Room::setSouth (Room * ptr)
{
  mSouth = ptr;
}

void Room::setEast (Room * ptr)
{
  mEast = ptr;
}

void Room::setWest (Room * ptr)
{
  mWest = ptr;
}

void Room::setParent (Room * ptr)
{
	mParent = ptr;
}

void Room::setGroupID (int id)
{
  mGroupID = id;
}

int Room::getGroupID ()
{
  return (mGroupID);
}



// --------------
// Wall
// --------------

Wall::Wall (Room * first, Room * second):
mFirst (first), mSecond (second), mPresent (true)
{
}

Room *
Wall::getFirst ()
{
  return (mFirst);
}

Room *
Wall::getSecond ()
{
  return (mSecond);
}