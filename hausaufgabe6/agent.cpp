/**********************************************************************\
* Dateiname: agent.cpp
* Autor : Mario Grotschar
		  Gerardo Martinez
		  Christoph Eder 
* Projekt : Projekt 3 Die Besucher
* Copyright (C) <<COPYRIGHT>>
*
* Kurzbeschreibung: Beinhaltet die Logik des Agenten in Form einer
*					Statemachine. Enthält auch die funktion startAgent() 
*					die im Main ausgeführt wird.
*
* Datum: Autor: Grund der Aenderung:
* 7.12.2011 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/

/*--- #includes der Form <...> ---------------------------------------*/

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

/*--- #includes der Form "..." ---------------------------------------*/

#include "stlastar.h"
#include "agent.h"

/*--- #define-Konstanten und Makros ----------------------------------*/
#define	AGENT_INIT_ERROR	10
#define	AGENT_ESTIMATE_ERROR	20

using namespace std;



bool operator==(const Agent& x, const Agent& y)
{
	return (x.aID == y.aID);
}

/**********************************************************************\
* Funktionsname: signalContact
*
* Kurzbeschreibung: Die einzige art der Kommunikation. Das ist nötig, da 
* es sonst passieren kann, dass ein Roboter einen anderen beruehrt dieser
* den Beruehrer aber nicht. Das ist die Folge eines sequentiellen Ablaufs 
* der Roboter innerhalb einer Iteration
*
\**********************************************************************/

void	Agent::signalContact(Agent *proximityAgent)
{
	visitedAgentsIterator tempIt;
	
	if (!Agent::alreadyTouched(proximityAgent))
	{
		// Add this robot to the visted List
		tempIt = findVisitedAgent(proximityAgent);
							
		Agent::visitedAgentsList.push_back(proximityAgent);
		if (tempIt != notvisitedAgents.end())
		{
			notvisitedAgents.erase(tempIt);
		}
			
	}
}

/**********************************************************************\
* Funktionsname: inFrame
*
* Kurzbeschreibung: Diese Funktion ueberpruef, ob sich das Ziel noch 
* innerhalb eines gewissen Frames von der Ursprungsposition befindet.
*
\**********************************************************************/

bool	Agent::inFrame(Agent* targetAgent, Room* targetRoom, float targetFrame)
{
	Agent dummyAgent;
	dummyAgent.currentRoom = targetRoom;

	if (getEstimateDistance(targetAgent, &dummyAgent) <= targetFrame)
		return true;
	else
		return false;
}

/**********************************************************************\
* Funktionsname: calculateFrame
*
* Kurzbeschreibung: Diese Funktion berechnet den Frame. Er ist direkt 
* proportional zur Entfernung des Ziels. Je weiter desto weniger Gewicht
* haben Fehler. Je naeher ich dme Ziel bin, umso gneauer muss geplant
* werden.
*
\**********************************************************************/

void	Agent::calculateFrame(Room* targetRoom)
{
	Agent dummyAgent;
	dummyAgent.currentRoom = targetRoom;
	fsearchFrame = getEstimateDistance(this, &dummyAgent);
	fsearchFrame /= 2;
}

/**********************************************************************\
* Funktionsname: keepDistance
*
* Kurzbeschreibung: Gewichtet alle adjazenten Räume mit der Summe der Distanzen
* zu allen anderen Agenten. Als Ziel wird der Raum mit der groeßten Summe
* gewaehlt. Die Idee ist es eine maximale Distanz zu allen Agenten zu halten.
*
\**********************************************************************/

void Agent::keepDistance()
{
	// This function keeps the max distance to all other agents. It therefore calculates the
	// heuristic f+g from all adjacent nodes to all other agents
	map <float, Room *> hxRoom;
	map <float, Room *>::iterator hxRoomIterator;
	pair <float, Room*> hxRoomInsert;
	float distanceHxNorth = 0;
	float distanceHxSouth = 0;
	float distanceHxWest = 0;
	float distanceHxEast = 0;
	
	Agent dummyAgent;
	
	(void) updateDistanceList();
	(void) plannedPath.clear();
	
	mapDistanceIterator mapIt;
	mapIt = distanceHeuristic.begin();
	
	// Insert Default Room if all adjacent fields are occupied.
	hxRoomInsert.first = -1;
	hxRoomInsert.second = this->currentRoom;
	hxRoom.insert(hxRoomInsert);

	visitedAgentsIterator tempIt;
	// Check if there is an agent in an adjacent room.
	if (currentRoom->hasEast())
	{
		if (currentRoom->mEast->occupiedRobot == NULL)
		{
			mapIt = distanceHeuristic.begin();
			dummyAgent.currentRoom = currentRoom->getEast();
			while (mapIt != distanceHeuristic.end())
			{
				distanceHxEast += getEstimateDistance(&dummyAgent, mapIt->first);
				mapIt++;
			}	

			hxRoomInsert.first = (float)distanceHxEast;
			hxRoomInsert.second = currentRoom->mEast;
			hxRoom.insert(hxRoomInsert);
		}
	}
				
	if (currentRoom->hasNorth())
	{
		if (currentRoom->mNorth->occupiedRobot == NULL)
		{
			mapIt = distanceHeuristic.begin();
			dummyAgent.currentRoom = currentRoom->getNorth();
			while (mapIt != distanceHeuristic.end())
			{
				distanceHxNorth += getEstimateDistance(&dummyAgent, mapIt->first);
				mapIt++;
			}	
			hxRoomInsert.first = (float)distanceHxNorth;
			hxRoomInsert.second = currentRoom->mNorth;
			hxRoom.insert(hxRoomInsert);
		}
	}
				
	if (currentRoom->hasSouth())
	{
		if (currentRoom->mSouth->occupiedRobot == NULL)
		{
			mapIt = distanceHeuristic.begin();
			dummyAgent.currentRoom = currentRoom->getSouth();
			while (mapIt != distanceHeuristic.end())
			{
				distanceHxSouth += getEstimateDistance(&dummyAgent, mapIt->first);
				mapIt++;
			}	
			hxRoomInsert.first = (float)distanceHxSouth;
			hxRoomInsert.second = currentRoom->mSouth;
			hxRoom.insert(hxRoomInsert);
		}
	}
				
	if (currentRoom->hasWest())
	{
		if (currentRoom->mWest->occupiedRobot == NULL)
		{
			mapIt = distanceHeuristic.begin();
			dummyAgent.currentRoom = currentRoom->getWest();
			while (mapIt != distanceHeuristic.end())
			{
				distanceHxWest += getEstimateDistance(&dummyAgent, mapIt->first);
				mapIt++;
			}
		
			hxRoomInsert.first = (float)distanceHxWest;
			hxRoomInsert.second = currentRoom->mWest;
			hxRoom.insert(hxRoomInsert);
		}
	}

	// Get last element. Due to the weak ordering it is the node with the greatest distance to all other agents.
	try
	{
		hxRoomIterator = hxRoom.end();
		hxRoomIterator--;

	}
	catch (...)
	{
		cout << "Error in Keep distance.";
	}

	plannedPath.push_back(hxRoomIterator->second);
	plannedPathIterator = plannedPath.begin();
	// Endof Robot check for visted agents
}

/**********************************************************************\
* Funktionsname: getClosestTarget
*
* Kurzbeschreibung: Findet den naehersten Agenten, welcher nicht 
* mit oldTarget uebereinstimmt.
*
\**********************************************************************/

Agent*	Agent::getClosestTarget(Agent* oldTarget)
{
	float tempValue = AGENT_MAXSEARCH;
	mapDistanceIterator tempIt;
	mapDistanceReturnPair tempReturn;
	Agent* returnValue = NULL;

	tempIt = distanceHeuristic.begin();

	while (tempIt != distanceHeuristic.end())
	{
		if ((tempIt->second < tempValue) && (tempIt->first != this) && (tempIt->first != oldTarget) && (!alreadyTouched(tempIt->first)))
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

/**********************************************************************\
* Funktionsname: getClosestTarget
*
* Kurzbeschreibung: Findet den naehersten Agenten, welcher nicht 
* in der Liste oldTarget ist. Ueberladung zur vorhergegangen funktion.
*
\**********************************************************************/
Agent*	Agent::getClosestTarget(listAgents &oldTarget)
{
	float tempValue = AGENT_MAXSEARCH;
	mapDistanceIterator tempIt;
	mapDistanceReturnPair tempReturn;
	Agent* returnValue = NULL;

	tempIt = distanceHeuristic.begin();

	while (tempIt != distanceHeuristic.end())
	{
		if ((tempIt->second < tempValue) && (tempIt->first != this) && (!agentInVector(tempIt->first,oldTarget) && (!alreadyTouched(tempIt->first))))
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

/**********************************************************************\
* Funktionsname: getTarget
*
* Kurzbeschreibung: getter Methode fuer das aktuelle Ziel.
*
\**********************************************************************/
Agent* Agent::getTarget()
{
	return targetAgent;
}

/**********************************************************************\
* Funktionsname: agentInVector
*
* Kurzbeschreibung: Untersucht ob ein Agent in der uebergebenen Liste ist.
*
\**********************************************************************/
bool	Agent::agentInVector (Agent * contactAgent, listAgents &searchVector)
{
	listAgentsIterator resultIterator;

	if (contactAgent == NULL)
	return false;

	resultIterator = find (searchVector.begin(), searchVector.end(), contactAgent);

	if (resultIterator != searchVector.end())
	{
		return (true);
	}
	else
	{
		return (false);
	}
}

/**********************************************************************\
* Funktionsname: alreadyTouched
*
* Kurzbeschreibung: Untersucht ob ein Agent bereits abgefahren wurde.
*
\**********************************************************************/
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


/**********************************************************************\
* Funktionsname: findVisitedAgent
*
* Kurzbeschreibung: Sucht nach Agenten in notvisitedAgents List.
*
\**********************************************************************/
visitedAgentsIterator Agent::findVisitedAgent (Agent * contactAgent)
{
	// search in visited list if robot was already touched.
	/* http://stackoverflow.com/questions/571394/how-to-find-an-item-in-a-stdvector */
	
	visitedAgentsIterator resultIterator;

	resultIterator = find (notvisitedAgents.begin(), notvisitedAgents.end(), contactAgent);

	return (resultIterator);
}

/**********************************************************************\
* Funktionsname: planPath
*
* Kurzbeschreibung: Plant den Weg zum Ziel mittels A-Star.
*
\**********************************************************************/

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

/**********************************************************************\
* Funktionsname: setNext
*
* Kurzbeschreibung: Trivial.
*
\**********************************************************************/
void Agent::setNext(Room* setRoom)
{
	nextRoom = setRoom;
}


/**********************************************************************\
* Funktionsname: pushToNotVisitedList
*
* Kurzbeschreibung: Trivial.
*
\**********************************************************************/
void Agent::pushToNotVisitedList(Agent* setTarget)
{
	notvisitedAgents.push_back(setTarget);
}


/**********************************************************************\
* Funktionsname: setTarget
*
* Kurzbeschreibung: Trivial.
*
\**********************************************************************/
void Agent::setTarget(Agent* setTarget)
{
	targetAgent = setTarget;
}

/**********************************************************************\
* Funktionsname: scanAgents
*
* Kurzbeschreibung: Ueberpruft alle adajzenten Raeume auf Agenten die 
* noch nicht abgefahren wurden.
*
\**********************************************************************/
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
				currentRoom->mEast->occupiedRobot->signalContact(this);
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mEast->occupiedRobot);
							
				Agent::visitedAgentsList.push_back(currentRoom->mEast->occupiedRobot);
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}
			
			}

			//// NEW STUFF
			///*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mEast->hasNorth())
			//{
			//	if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mEast->mNorth->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mEast->mNorth->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mEast->mNorth->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())

			//			/*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mEast->hasSouth())
			//{
			//	if (currentRoom->mEast->mSouth->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mEast->mSouth->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mEast->mSouth->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mEast->mSouth->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())
		} // Endif if (!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
	}
				
	if (currentRoom->hasNorth())
	{
		if (currentRoom->mNorth->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mNorth->occupiedRobot))
			{
				currentRoom->mNorth->occupiedRobot->signalContact(this);
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mNorth->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mNorth->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}


			}

			//			// NEW STUFF
			///*
			//*   OOX
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mNorth->hasWest())
			//{
			//	if (currentRoom->mNorth->mWest->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mNorth->mWest->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mNorth->mWest->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mNorth->mWest->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())

			//			/*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mNorth->hasEast())
			//{
			//	if (currentRoom->mNorth->mEast->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mNorth->mEast->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mNorth->mEast->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mNorth->mEast->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())
		}
	}
				
	if (currentRoom->hasSouth())
	{
		if (currentRoom->mSouth->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mSouth->occupiedRobot))
			{
				currentRoom->mSouth->occupiedRobot->signalContact(this);
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mSouth->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mSouth->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}
				

			}

			//					// NEW STUFF
			///*
			//*   OOX
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mSouth->hasWest())
			//{
			//	if (currentRoom->mSouth->mWest->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mSouth->mWest->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mSouth->mWest->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mSouth->mWest->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())

			//			/*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mSouth->hasEast())
			//{
			//	if (currentRoom->mSouth->mEast->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mSouth->mEast->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mSouth->mEast->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mSouth->mEast->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())
		}
	}
				
	if (currentRoom->hasWest())
	{
		if (currentRoom->mWest->occupiedRobot != NULL)
		{
			if (!Agent::alreadyTouched(currentRoom->mWest->occupiedRobot))
			{
				currentRoom->mWest->occupiedRobot->signalContact(this);
				// Add this robot to the visted List
				tempIt = findVisitedAgent(currentRoom->mWest->occupiedRobot);
				Agent::visitedAgentsList.push_back(currentRoom->mWest->occupiedRobot);
						
				if (tempIt != notvisitedAgents.end())
				{
					notvisitedAgents.erase(tempIt);
				}
			}

			//		// NEW STUFF
			///*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mWest->hasNorth())
			//{
			//	if (currentRoom->mWest->mNorth->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mWest->mNorth->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mWest->mNorth->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mWest->mNorth->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())

			//			/*
			//*   XOO
			//*   ORO
			//*   OOO
			//*
			//*   R ... Current Robot
			//*   X ... adjacent field to check. North-East
			//*/
			//if (currentRoom->mWest->hasSouth())
			//{
			//	if (currentRoom->mWest->mSouth->occupiedRobot != NULL)
			//	{
			//		if (!Agent::alreadyTouched(currentRoom->mWest->mSouth->occupiedRobot))
			//		{
			//			// Add this robot to the visted List
			//			tempIt = findVisitedAgent(currentRoom->mWest->mSouth->occupiedRobot);
			//				
			//			Agent::visitedAgentsList.push_back(currentRoom->mWest->mSouth->occupiedRobot);
			//			if (tempIt != notvisitedAgents.end())
			//			{
			//				notvisitedAgents.erase(tempIt);
			//			}
			//		} // Endif if(!Agent::alreadyTouched(currentRoom->mEast->occupiedRobot))
			//	} // Endif if (currentRoom->mEast->mNorth->occupiedRobot != NULL)
			//} // Endif if (currentRoom->mEast->hasNorth())
		}
	}
	// Endof Robot check for visted agents
}

/**********************************************************************\
* Funktionsname: moveAgent
*
* Kurzbeschreibung: Bewegt den Agenten in den vom Planungsalgorithmus 
* berechneten naechsten Raum. Im Falle einer Kollision wird der State
* entsprechend gesetzt (targetState).
*
\**********************************************************************/
void Agent::moveAgent(AgentState targetState)
{
	// Room is free.
	if (!(plannedPathIterator == plannedPath.end()))
	{
		if ((*plannedPathIterator)->occupiedRobot != NULL)
		{
			// Room is occupied.
			nextState = targetState;
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

}

/**********************************************************************\
* Funktionsname: getEstimateDistance
*
* Kurzbeschreibung: Berechnet die Manhattendistanz vom solverAgent zum
* targetAgent.
*
\**********************************************************************/
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

/**********************************************************************\
* Funktionsname: updateDistanceList
*
* Kurzbeschreibung: Berechnet die Manhattendistanzen zu allen anderen
* Agenten.
*
\**********************************************************************/
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

/**********************************************************************\
* Funktionsname: initDistanceMap
*
* Kurzbeschreibung: Initialsiert die Manhattendistanzen zu allen anderen
* Agenten.
*
\**********************************************************************/
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

/**********************************************************************\
* Funktionsname: setnotvisitedAgents
*
* Kurzbeschreibung: Trivial
*
\**********************************************************************/
void	Agent::setnotvisitedAgents(vector <Agent *> setnotvisited)
{
	notvisitedAgents = setnotvisited;
}

/**********************************************************************\
* Funktionsname: isSearchComplete
*
* Kurzbeschreibung: Trivial 
*
\**********************************************************************/
bool	Agent::isSearchComplete()
{
	return bsearchComplete;
}

/**********************************************************************\
* Funktionsname: startAgent
*
* Kurzbeschreibung: Ist die Hauptfunktion von agent.cpp. Diese wird auch
* in der Mainfunktion aufgerufen, um den naechsten Schritt des Agenten
* auszufuehren. Kern der Funktion sind zwei Statemachines, die die verschiedenen
* Strategien widerspiegeln. Die erste Statemachine hat die Funktion das Ver-
* halten waehrend der suche zu steuern. Die Zweite wird verwendet, wenn die 
* Suche beendet wurde.
*
\**********************************************************************/
unsigned int Agent::startAgent()
{
	// Agent State machine
	visitedAgentsIterator tempIt;
	listAgents exclusionVector;
	Agent* oldTarget;
	bool searchFalg = false;

	if (!bsearchComplete)
	{
		switch(currentState)
		{
				case AGENT_STATE_NOT_INITIALISED:
	
					// Plan Path.
					Agent::nextState = AGENT_STATE_REPLAN;
				
					// Delete own reference in not visited list.
					tempIt = findVisitedAgent(this);
				
					if (tempIt != notvisitedAgents.end())
					{
						notvisitedAgents.erase(tempIt);
					}

				
					// Init distance map.
					try
					{
						if (!initDistanceMap())
							throw AGENT_INIT_ERROR;
					}
					catch (int e)
					{
						cout << "An exception occurred. Exception Nr. " << e << endl;
						nextState = AGENT_STATE_NOT_INITIALISED;
					}

					// Set prime marker (pointer to robot and flag) on maze.
					try
					{
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
						 Agent::nextState = AGENT_STATE_NOT_INITIALISED;
					}

					// Get first target.
					
					(void) updateDistanceList();
					targetAgent = getClosestTarget(targetAgent);
					targetRoom = targetAgent->currentRoom;
			

				break;
 
				case AGENT_STATE_SEARCH_MODE: 
					//(void) updateDistanceList();
					(void) scanAgents();
					if (Agent::notvisitedAgents.empty())
					{
						// No more robots left in queue.
						Agent::nextState = AGENT_STATE_COMPLETE;
						break;
					}


					// run strategy to find other robots
					if (Agent::plannedPathIterator == plannedPath.end())
					{
						Agent::nextState = AGENT_STATE_REPLAN;

					}
					else //von if (Agent::plannedPath.empty())
					{
						//Agent::nextState = AGENT_STATE_REPLAN;
						 // TODO: Check if target moved. Recalculate or so.d.
						if (targetRoomOld && targetAgent )
						{
							calculateFrame(targetRoomOld);
							if  (!inFrame(targetAgent, targetRoomOld, fsearchFrame))
								Agent::nextState = AGENT_STATE_REPLAN;
						}
					}

					(void) moveAgent(AGENT_STATE_COLLISION);

					// Verify if search is complete.

				
				break;

				case AGENT_STATE_REPLAN:
					(void) scanAgents();
					if (Agent::notvisitedAgents.empty())
					{
						// No more robots left in queue.
						Agent::nextState = AGENT_STATE_COMPLETE;
						break;
					}

					(void) updateDistanceList();
					(void) plannedPath.clear();
					(void) exclusionVector.clear();
			

					exclusionVector.push_back(this);
					oldTarget = targetAgent;

					if (targetAgent)
						if (alreadyTouched(targetAgent))
							exclusionVector.push_back(targetAgent);
				
					nextState = AGENT_STATE_SEARCH_MODE;

					do
					{
						// Try to find a new target, which is closet to agent. Ignores Agents in exclusionVector.
						targetAgent = getClosestTarget(exclusionVector);
						if (targetAgent)
						{
							targetRoom = targetAgent->currentRoom;
							if (planPath(targetRoom))
							{	
								plannedPathIterator = plannedPath.begin();
								searchFalg = true;
							}
						}
						else
						{
							// No untouched targets are not reachable. Get some distance for now.
							targetAgent = oldTarget;
							targetRoom = targetAgent->currentRoom;
							searchFalg = true;
							//nextState = AGENT_STATE_REPLAN; // This should be changed to another state to get some distance from the other agents.
							nextState = AGENT_STATE_DISTANCE_MODE;
						}
							// we need to wait until any target gets reachable
						exclusionVector.push_back(targetAgent);

					} while (!searchFalg);

					if (Agent::notvisitedAgents.empty())
					{
							// No more robots left in queue.
							Agent::nextState = AGENT_STATE_COMPLETE;
					}

					//if (currentState == AGENT_STATE_COMPLETE)
					//	nextState = AGENT_STATE_COMPLETE;
				break;
			
				case AGENT_STATE_COLLISION:
					// Handle if robot detects another robot in an adjacent field.
					// TODO: Check if adjacent robot is goal, any other robot not visited or alread visited robot.
					
					(void) scanAgents();
					if (Agent::notvisitedAgents.empty())
					{
						// No more robots left in queue.
						Agent::nextState = AGENT_STATE_COMPLETE;
						break;
					}

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

						nextState =	AGENT_STATE_REPLAN;
					}

						//if (currentState == AGENT_STATE_COMPLETE)
						//		nextState = AGENT_STATE_COMPLETE;
				break;

				case AGENT_STATE_DISTANCE_MODE:
					(void) scanAgents();
					
					if (Agent::notvisitedAgents.empty())
					{
						// No more robots left in queue.
						Agent::nextState = AGENT_STATE_COMPLETE;
						break;
					}
					
					if (mDistanceCount++ >= AGENT_MAXWAIT)
					{
						mDistanceCount = 0;
						nextState = AGENT_STATE_SEARCH_MODE;
					}
					else
						nextState = AGENT_STATE_DISTANCE_MODE;
					
					keepDistance();
					moveAgent(AGENT_STATE_DISTANCE_MODE);
					
				break;

				case AGENT_STATE_COMPLETE:

					// Change Strategy. Also change Statemachine. ( This is done by the if(bsearchComplete) statement)
					bsearchComplete = true;
					Agent::nextState = AGENT_STATE_COMPLETE;


				break;
		} // End of switch statement.
	} // End of if statement.
	else
	{
		// Statemachine for the final strategy.
		switch(currentState)
		{
			case AGENT_STATE_COMPLETE:
				nextState = AGENT_STATE_COMPLETE;
				/* Putting some entropy into the system.
				 Without this deadlocks could occur. */
				if (rand() % 10 < 7)
				{
					keepDistance();
					moveAgent(AGENT_STATE_NOP);
				}
				
			break;

			case	AGENT_STATE_NOP:
				//Do nothing. Just calculate a heuristic step.
				nextState = AGENT_STATE_COMPLETE;

			break;
		}
	}
	//if (currentState == AGENT_STATE_COMPLETE)
	//	nextState = AGENT_STATE_COMPLETE;

	targetRoomOld = targetRoom;
	currentState = nextState;
	return currentState;
}

