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

#include "maze.h"
#include "planner.h"




class Agent
{

public:

	// Konstruktor
	Agent(int setID = 0, Room * setRoom = NULL, Room * setNextRoom = NULL) : aID (setID), currentRoom (setRoom), nextRoom(setNextRoom) {};

	// get ID
	int	getID() { return aID;}
	void	setID(int setID) {aID = setID;};

	void	setTarget(Agent setTarget); 
	// get Raum
	Room*	getNext() { return nextRoom;}
	Room*	getCurrent() { return currentRoom;}
		// Pointer auf aktuellen Raum
	Room*	currentRoom;
	
	// Pointer auf nächsten Raim
	Room*	nextRoom;
	// set Room
	void	setNext (Room* setRoom);

	// Agent bewegen
	Room*	moveAgent();

protected:

private:
	int	aID;

	// Set besuchte Agenten
	set	<Agent>	visitedAgents;

	// Vektor noch nicht besuchte Agenten
	vector <Agent> notvisitedAgents;

	// Gegangener Weg
	// Koordinaten
	// ID
};

#endif //_INCL_AGENT










