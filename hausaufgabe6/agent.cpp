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

using namespace std;

bool operator==(const Agent& x, const Agent& y)
{
	return (x.aID == y.aID);
}

bool Agent::alreadyTouched (Agent * contactAgent)
{
	// search in visited list if robot was already touched.
	/* http://stackoverflow.com/questions/571394/how-to-find-an-item-in-a-stdvector */
	visitedAgents ag;
	visitedAgentsIterator resultIterator;

	resultIterator = find (ag.begin(), ag.end(), contactAgent);

	if (resultIterator != ag.end())
	{
		return (true);
	}
	else
	{
		return (false);
	}
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
		nodeEnd.room =  targetRoom;// targetAgent->currentRoom; // TODO: Use TargetAgent instead? Or set targetRoom when selecting targetAgent

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

void Agent::setTarget(Agent* setTarget)
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

void	Agent::setnotvisitedAgents(vector <Agent *> setnotvisited)
{
	notvisitedAgents = setnotvisited;
}

Agent::AgentState Agent::startAgent()
{
	// Agent State machine
	switch(currentState)
	{
			case AGENT_STATE_NOT_INITIALISED:
				// Delete own reference in not visited list.
				//notvisitedAgents.fi

				Agent::nextState = AGENT_STATE_NOT_INITIALISED;

				// TODO check before planning. change to target Agent type instead of Room.
				Agent::planPath(targetRoom);
				
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

				//if (targetRoom != NULL) // change target Room to agent
				//		{
				//			// Plan initial path.
				//			if (targetRoom/*targetRoom->occupiedRobot*/)
				//			{
				//				//planPath(targetRoom->occupiedRobot);
				//				Agent::planPath(targetRoom);
				//			}
				//			else
				//			{
				//				Agent::nextState = AGENT_STATE_NOT_INITIALISED;
				//			}
				//		}

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
						
						nextState = AGENT_STATE_COMPLETE;
					}
				}
				
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

