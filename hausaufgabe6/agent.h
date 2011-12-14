/**********************************************************************\
* Dateiname: agent.h
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

#ifndef _INCL_AGENT
#define _INCL_AGENT

// stl includes
#include <algorithm>
#include <set>
#include <vector>
#include <list>

#include "maze.h"
#include "planner.h"


class Agent
{
	friend bool operator==(const Agent&, const Agent&);

	enum AgentState
	{
		AGENT_STATE_NOT_INITIALISED,
		AGENT_STATE_SEARCH_MODE,
		AGENT_STATE_COLLISION,
		AGENT_STATE_COMPLETE
	};

public:

	// Konstruktor
	Agent(int setID = 0, Room * setRoom = NULL, Room * setNextRoom = NULL, AgentState setState = AGENT_STATE_NOT_INITIALISED) : aID (setID), currentRoom (setRoom), nextRoom(setNextRoom), currentState(setState) {};

	// get ID
	int	getID() { return aID;}

	void	setID(int setID) {aID = setID;};
	void	setTarget(Agent* setTarget); 
	Agent*	getTarget(); 
	void	setCurrentState(AgentState setState);
	void	setNext (Room* setRoom);
	void	setnotvisitedAgents(vector <Agent *> setnotvisited);

	// get Raum
	Room*	getNext() { return nextRoom;}
	Room*	getCurrent() { return currentRoom;}
	
	// Pointer auf aktuellen Raum
	Room*	currentRoom;
	Room*	targetRoom;
	
	// Pointer auf nächsten Raum
	Room*	nextRoom;
	// set Room


	// Agent bewegen
	Room*	moveAgent();

	AgentState startAgent();

	
	// Geplanter Weg
	typedef list <Room *> listPath;
	listPath	plannedPath;
	typedef listPath::iterator listPathIterator;
	listPathIterator plannedPathIterator;	
	
	// Set besuchte Agenten
	vector	<Agent *>	visitedAgents;

	// Vektor noch nicht besuchte Agenten
	vector <Agent *> notvisitedAgents;




protected:

private:

	int	aID;

	AgentState currentState;
	AgentState nextState;




	// Gegangener Weg. TODO: Brauchen wir den
	// Koordinaten
	// ID
};

#endif //_INCL_AGENT










