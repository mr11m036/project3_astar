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

#define	AGENT_INIT_ERROR	10
#define	AGENT_ESTIMATE_ERROR	20

using namespace std;

bool operator==(const Agent& x, const Agent& y)
{
	return (x.aID == y.aID);
}

Agent*	Agent::getClosestTarget(Agent* oldTarget)
{
	float tempValue = AGENT_MAXSEARCH;
	mapDistanceIterator tempIt;
	mapDistanceReturnPair tempReturn;
	Agent* returnValue = NULL;

	tempIt = distanceHeuristic.begin();

	while (tempIt != distanceHeuristic.end())
	{
		if ((tempIt->second < tempValue) && (tempIt->first != this) && (tempIt->first != oldTarget))
		{
			tempValue = tempIt->second;
			// get Agent. Its niot itselft tbecause of if clause.
			returnValue = tempIt->first;
		}
		else
		{
			tempValue = tempValue;
		}

		tempIt++;
	}

	return	returnValue;
 
}

Agent* Agent::getTarget()
{
	return targetAgent;
}

bool Agent::alreadyTouched (Agent * contactAgent)
{
	// search in visited list if robot was already touched.
	/* http://stackoverflow.com/questions/571394/how-to-find-an-item-in-a-stdvector */
	
	visitedAgentsIterator resultIterator;

	if (contactAgent == NULL)
		return false;

	resultIterator = find (visitedAgentsList.begin(), visitedAgentsList.end(), contactAgent);

	if (resultIterator != visitedAgentsList.end())
	{
		return (true);
	}
	else
	{
		return (false);
	}
}

// TODO Delete Function erstellen mit Parameter Agent und Vector
visitedAgentsIterator Agent::findVisitedAgent (Agent * contactAgent)
{
	// search in visited list if robot was already touched.
	/* http://stackoverflow.com/questions/571394/how-to-find-an-item-in-a-stdvector */
	
	visitedAgentsIterator resultIterator;

	resultIterator = find (notvisitedAgents.begin(), notvisitedAgents.end(), contactAgent);

	return (resultIterator);
}



bool Agent::planPath(Room* targetAgent)
{
	AStarSearch<MapSearchNode> astarsearch;
	bool	searchResult = false;

	unsigned int SearchCount = 0;
	const unsigned int NumSearches = 1;
	
	while(SearchCount < NumSearches)
	{
	
		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.room = currentRoom; // Start from this robots location.

		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.room =  targetAgent;// targetAgent->currentRoom; // TODO: Use TargetAgent instead? Or set targetRoom when selecting targetAgent

		// Set Start and goal states
		
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

		unsigned int SearchState;
		unsigned int SearchSteps = 0;

		do
		{
			SearchState = astarsearch.SearchStep();

			SearchSteps++;

	#if DEBUG_LISTS

			cout << "Steps:" << SearchSteps << "\n";

			int len = 0;

			cout << "Open:\n";
			MapSearchNode *p = astarsearch.GetOpenListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY			
				((MapSearchNode *)p)->PrintNodeInfo();
	#endif
				p = astarsearch.GetOpenListNext();
				
			}

			cout << "Open list has " << len << " nodes\n";

			len = 0;

			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY			
				p->PrintNodeInfo();
	#endif			
				p = astarsearch.GetClosedListNext();
			}

			cout << "Closed list has " << len << " nodes\n";
	#endif

		}
		while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			//cout << "Search found goal state\n";
				searchResult = true;
				MapSearchNode *node = astarsearch.GetSolutionStart();

	#if DISPLAY_SOLUTION
				//cout << "Displaying solution\n";
	#endif
				int steps = 0;

				node->PrintNodeInfo();
				for( ;; )
				{
					node = astarsearch.GetSolutionNext();

					if( !node )
					{
						break;
					}

					// Setzt Pfadinfo  
					//node->PrintNodeInfo();

					// Save path to agent.
					
					plannedPath.push_back(node->getRoom());

					steps ++;
				
				};

				//cout << "Solution steps " << steps << endl;

				// Once you're done with the solution you can free the nodes up
				astarsearch.FreeSolutionNodes();

	
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			//cout << "Search terminated. Did not find goal state\n";
			searchResult = false;
		
		}

		// Display the number of loops the search went through
		//cout << "SearchSteps : " << SearchSteps << "\n";

		SearchCount ++;

		astarsearch.EnsureMemoryFreed();
	}

	return (searchResult);
}

void Agent::setNext(Room* setRoom)
{
	nextRoom = setRoom;
}

void Agent::pushToNotVisitedList(Agent* setTarget)
{
	notvisitedAgents.push_back(setTarget);
}

void Agent::setTarget(Agent* setTarget)
{
	targetAgent = setTarget;
}

void Agent::scanAgents()
{
	visitedAgentsIterator tempIt;
	// Check if there is an agent in an adjacent room.
	if (currentRoom->hasEast())
	{
		if (currentRoom->mEast->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			{
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mEast->occupiedRobot);
							
				Agent::visitedAgentsList.push_back(currentRoom->mEast->occupiedRobot);
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}

				//if (currentRoom->mEast->occupiedRobot == targetAgent)
				//{
				//	// Erase the current target. We foudn it.
				//	targetAgent = NULL;
				//	targetRoom = this->currentRoom;
				//}
			
			}
		}
	}
				
	if (currentRoom->hasNorth())
	{
		if (currentRoom->mNorth->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mNorth->occupiedRobot))
			{
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mNorth->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mNorth->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}

				//if (currentRoom->mNorth->occupiedRobot == targetAgent)
				//{
				//	// Erase the current target. We foudn it.
				//	targetAgent = NULL;
				//	targetRoom = this->currentRoom;
				//}
			}
		}
	}
				
	if (currentRoom->hasSouth())
	{
		if (currentRoom->mSouth->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mSouth->occupiedRobot))
			{
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mSouth->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mSouth->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}
				
				//if (currentRoom->mSouth->occupiedRobot == targetAgent)
				//{
				//	// Erase the current target. We foudn it.
				//	targetAgent = NULL;
				//	targetRoom = this->currentRoom;
				//}
			}
		}
	}
				
	if (currentRoom->hasWest())
	{
		if (currentRoom->mWest->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mWest->occupiedRobot))
			{
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mWest->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mWest->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}

				//if (currentRoom->mWest->occupiedRobot == targetAgent)
				//{
				//	// Erase the current target. We foudn it.
				//	targetAgent = NULL;
				//	targetRoom = this->currentRoom;
				//}
			}
		}
	}
	// Endof Robot check for visted agents
}

void Agent::moveAgent()
{
	// Room is free.
	if (!(plannedPathIterator == plannedPath.end()))
	{
		if ((*plannedPathIterator)->occupiedRobot != NULL)
		{
			// Room is occupied.
			nextState = AGENT_STATE_COLLISION;
		}
		else
		{
			Agent::currentRoom->mFlagAstar = false; // required for now to print the agent.
			Agent::currentRoom->occupiedRobot = NULL;

			Agent::currentRoom = *plannedPathIterator;

			Agent::currentRoom->mFlagAstar = true;
			Agent::currentRoom->occupiedRobot = this;

			plannedPathIterator++;
		}
	}
	else
	{
						
		nextState = AGENT_STATE_REPLAN;
	}
}

float	Agent::getEstimateDistance (Agent* solverAgent ,Agent* targetAgent)
{
	float returnvalue = -1;
	try
	{
		if ((targetAgent == NULL) || (targetAgent->currentRoom == NULL))
			throw AGENT_ESTIMATE_ERROR;
		
		float xd = fabs(float(((float)solverAgent->currentRoom->mColumn - (float)targetAgent->currentRoom->mColumn)));
		float yd = fabs(float(((float)solverAgent->currentRoom->mRow - (float)targetAgent->currentRoom->mRow)));
		returnvalue = xd + yd;
	}
	catch (int e1)
	{
		returnvalue = -1;
	}

	return returnvalue;
}

void	Agent::updateDistanceList()
{
	mapDistanceIterator mapIt;

	mapIt = distanceHeuristic.begin();

	while (mapIt != distanceHeuristic.end())
	{
		mapIt->second = getEstimateDistance(this, mapIt->first);
		mapIt++;
	}

}

bool	Agent::initDistanceMap()
{
	bool returnval = false;
	listAgentsIterator tempIt;
	float tempDistance = 0;
	mapDistanceReturnPair retPair;
	
	if (!notvisitedAgents.empty())
	{
		tempIt = notvisitedAgents.begin();
		distanceHeuristicIt = distanceHeuristic.begin();
		while (tempIt != notvisitedAgents.end())
		{
			tempDistance = getEstimateDistance(this, *tempIt);
			retPair = distanceHeuristic.insert(mapDistanceInsertPair(*tempIt, (float)tempDistance));
			tempIt++;
		}
		returnval = true;
	}
	else
	{
		returnval = false;
	}

	return returnval;
}

void	Agent::setnotvisitedAgents(vector <Agent *> setnotvisited)
{
	notvisitedAgents = setnotvisited;
}

int Agent::startAgent()
{
	// Agent State machine
	visitedAgentsIterator tempIt;

	switch(currentState)
	{
			case AGENT_STATE_NOT_INITIALISED:
				// Delete own reference in not visited list.
				//notvisitedAgents.fi
				tempIt = findVisitedAgent(this);
				// Init distance map.
				try
				{
					if (!initDistanceMap())
						throw AGENT_INIT_ERROR;
				}
				catch (int e)
				{
				}

				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}

				Agent::nextState = AGENT_STATE_NOT_INITIALISED;

				 //TODO check before planning. change to target Agent type instead of Room.
				if (targetAgent)
				{
					if (targetAgent->currentRoom)
					{
						targetRoom = targetAgent->currentRoom;
						Agent::planPath(targetRoom);
					}
				}
				else
				{
					// No target
					Agent::nextState = AGENT_STATE_NOT_INITIALISED;
				}
				//Agent::planPath(targetRoom);
				try
				{
					// Set prime marker (pointer to robot and flag) on maze.
					if ((currentRoom->occupiedRobot == NULL) || (currentRoom->occupiedRobot == this))
					{
						currentRoom->occupiedRobot = this;
					}
					else
					{
						// throw exception. Initial Place is occupied. This should not have happened.
						Agent::nextState = AGENT_STATE_NOT_INITIALISED;
						throw AGENT_INIT_ERROR;
					}
				}
				catch (int e)
				{
					 cout << "An exception occurred. Exception Nr. " << e << endl;
				}
				// set first target
				// plannedPath is alread next Room.

				if (!plannedPath.empty() && (currentRoom->occupiedRobot == this))
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
				//(void) updateDistanceList();
				(void) scanAgents();

				// run strategy to find other robots
				if (Agent::plannedPathIterator == plannedPath.end())
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
						Agent::nextState = AGENT_STATE_REPLAN;
					}

				}
				else //von if (Agent::plannedPath.empty())
				{
					//Agent::nextState = AGENT_STATE_REPLAN;
					 // TODO: Check if target moved. Recalculate or so.d.
				}

				(void) moveAgent();
				
			break;

			case AGENT_STATE_REPLAN:


				if (alreadyTouched(targetAgent))
				{
					// get next target
					(void) updateDistanceList();
					targetAgent = getClosestTarget(targetAgent);
					targetRoom = targetAgent->currentRoom;

					(void) plannedPath.clear();

					if (planPath(targetRoom))
						plannedPathIterator = plannedPath.begin();
					else
						plannedPathIterator = plannedPath.end();

				}
				else
				{
					// get a new path to current target
					(void) plannedPath.clear();
					targetRoom = targetAgent->currentRoom;

					if (planPath(targetRoom))
						plannedPathIterator = plannedPath.begin();
					else
						plannedPathIterator = plannedPath.end();
				}
			
				nextState = AGENT_STATE_SEARCH_MODE;


				
				
			break;
			
			case AGENT_STATE_COLLISION:
				// Handle if robot detects another robot in an adjacent field.
				// TODO: Check if adjacent robot is goal, any other robot not visited or alread visited robot.

					if (!(plannedPathIterator == plannedPath.end()))
					{
						//if ((*plannedPathIterator)->mFlagAstar)
						if ((*plannedPathIterator)->occupiedRobot != NULL)
						{
							// Room is occupied.
							if (mWaitCount++ >= AGENT_MAXWAIT)
							{
								mWaitCount = 0;
								nextState = AGENT_STATE_REPLAN;
							}
							else
								nextState = AGENT_STATE_COLLISION;
							
						}
						else
						{
							// Continue search mode.
							//mWaitCount = 0;
							nextState = AGENT_STATE_SEARCH_MODE;

						}
					}
					else
					{
						// if list is empty
						if (notvisitedAgents.empty())
						{
							nextState = AGENT_STATE_COMPLETE;
						}
						else
						{
							nextState =	AGENT_STATE_REPLAN;
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

