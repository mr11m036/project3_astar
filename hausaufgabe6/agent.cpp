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
