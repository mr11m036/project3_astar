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

Agent::AgentState Agent::startAgent()
{
	// Agent State machine
	switch(currentState)
	{
			case AGENT_STATE_NOT_INITIALISED:
				// set first target
				// plannedPath is alread next Room.
				plannedPathIterator = plannedPath.begin();
				Agent::nextState = AGENT_STATE_SEARCH_MODE;
			break;
 
			case AGENT_STATE_SEARCH_MODE: 
				// run strategy to find other robots
				if (Agent::plannedPath.empty())
				{
					// TODO: Check if some robots still in not visited Queue.
					//		If so set Other target
					//		If done set state to AGENT_STATE_COMPLETE
				}
				else
				{
					 // TODO: Check if target moved. Recalculate or so.
					 
					// Move to next room
					// TODO check if field is occupied.
	
					
					
					if (!(plannedPathIterator == plannedPath.end()))
					{
						Agent::currentRoom->mFlagAstar = false; // required for now to print the agent.
						Agent::currentRoom = *plannedPathIterator;
						Agent::currentRoom->mFlagAstar = true;
						plannedPathIterator++;
					}
					else
					{
						nextState = AGENT_STATE_COMPLETE;
					}

				}

			break;
 
			case AGENT_STATE_COMPLETE:
				// run strategy to evade other robots

			break;
	}

	currentState = nextState;
	return currentState;
}

