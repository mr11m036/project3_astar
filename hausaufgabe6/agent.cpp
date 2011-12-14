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

#include "stlastar.h"
#include "agent.h"


using namespace std;

void Agent::setNext(Room* setRoom)
{
	nextRoom = setRoom;
}

void Agent::setTarget(Agent setTarget)
{
	notvisitedAgents.push_back(setTarget);
}

Room* Agent::moveAgent()
{
	// TODO: check if next room is occupied.
	if (nextRoom != NULL)
	{
		currentRoom = nextRoom;
		return	currentRoom;
	}
	else
	{
		// Next room couldn't be set
		return NULL;
	}

}

void	Agent::setnotvisitedAgents(vector <Agent> setnotvisited)
{
	notvisitedAgents = setnotvisited;
}

Agent::AgentState Agent::startAgent()
{
	// Agent State machine
	switch(currentState)
	{
			case AGENT_STATE_NOT_INITIALISED:
				// set first target
				// plannedPath is alread next Room.
				if (!plannedPath.empty())
				{
					plannedPathIterator = plannedPath.begin();
					Agent::nextState = AGENT_STATE_SEARCH_MODE;
				}
				else
				{
					Agent::nextState = AGENT_STATE_NOT_INITIALISED;
				}
			break;
 
			case AGENT_STATE_SEARCH_MODE: 
				// run strategy to find other robots
				if (Agent::plannedPath.empty())
				{
					// TODO: Check if some robots still in not visited Queue.
					if (Agent::notvisitedAgents.empty())
					{
						// No more robots left in queue.
						Agent::nextState = AGENT_STATE_COMPLETE;
					}
					else
					{
						// TODO´: define new target
						//		  depends on strategy
					}

				}
				else //von if (Agent::plannedPath.empty())
				{
					 // TODO: Check if target moved. Recalculate or so.d.
					// Room is free.
					if (!(plannedPathIterator == plannedPath.end()))
					{
						if ((*plannedPathIterator)->mFlagAstar)
						{
							// Room is occupied.
							nextState = AGENT_STATE_COLLISION;
						}
						else
						{
							Agent::currentRoom->mFlagAstar = false; // required for now to print the agent.
							Agent::currentRoom = *plannedPathIterator;
							Agent::currentRoom->mFlagAstar = true;
							plannedPathIterator++;
						}
					}
					else
					{
						
						nextState = AGENT_STATE_COMPLETE;
					}
				}
				
			break;

			case AGENT_STATE_COLLISION:
				// Handle if robot detects another robot in an adjacent field.
				// TODO: Check if adjacent robot is goal, any other robot not visited or alread visited robot.

					if (!(plannedPathIterator == plannedPath.end()))
					{
						if ((*plannedPathIterator)->mFlagAstar)
						{
							// Room is occupied.
							nextState = AGENT_STATE_COLLISION;
						}
						else
						{
							// Continue search mode.
							nextState = AGENT_STATE_SEARCH_MODE;
						}
					}
					else
					{
						nextState = AGENT_STATE_COMPLETE;
					}

			break;

			case AGENT_STATE_COMPLETE:
				// run strategy to evade other robots
	

			break;
	}

	currentState = nextState;
	return currentState;
}

