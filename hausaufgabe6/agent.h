/**********************************************************************\
* Dateiname: agent.h
* Autor : Mario Grotschar
*		  Gerardo Martinez
*		  Christoph Eder 
*
* Projekt : Projekt 3 Die Besucher
* Copyright (C) <<COPYRIGHT>>
*
* Kurzbeschreibung: Headerfile fuer agent.cpp
*
* Datum: Autor: Grund der Aenderung:
* 7.12.2011 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/

#ifndef _INCL_AGENT
#define _INCL_AGENT



/*--- #includes der Form <...> ---------------------------------------*/
#include <algorithm>
#include <set>
#include <vector>
#include <list>

/*--- #includes der Form "..." ---------------------------------------*/
#include "maze.h"
#include "planner.h"

/*--- #define-Konstanten und Makros ----------------------------------*/
#define AGENT_MAXWAIT 4
#define	AGENT_MAXSEARCH 100

/*--- Datentypen (typedef) -------------------------------------------*/
typedef list <Room *> listPath;
typedef listPath::iterator listPathIterator;
typedef vector	<Agent *>	visitedAgents;
typedef visitedAgents::iterator visitedAgentsIterator;
typedef vector	<Agent *>	listAgents;
typedef visitedAgents::iterator listAgentsIterator;
typedef	map <Agent *, float> mapDistance;
typedef	mapDistance::iterator mapDistanceIterator;
typedef	pair<mapDistanceIterator, bool> mapDistanceReturnPair;
typedef pair <Agent *, float> mapDistanceInsertPair;



class Agent
{
	friend bool operator==(const Agent&, const Agent&);
	friend class MapSearchNode;

public:
	enum AgentState
	{
		AGENT_STATE_NOT_INITIALISED,
		AGENT_STATE_SEARCH_MODE,
		AGENT_STATE_COLLISION,
		AGENT_STATE_REPLAN,
		AGENT_STATE_DISTANCE_MODE,
		AGENT_STATE_COMPLETE,
		AGENT_STATE_NOP
	};

public:

	// Konstruktor
	Agent(int setID = 0, Room * setRoom = NULL, Room * setNextRoom = NULL, AgentState setState = AGENT_STATE_NOT_INITIALISED) : 
	aID (setID), 
	currentRoom (setRoom), 
	nextRoom(setNextRoom), 
	currentState(setState), 
	mWaitCount(0),
	mDistanceCount(0),
	bsearchComplete(false),
	fsearchFrame(0)
	{};

	// get ID
	int	getID() { return aID;}
	unsigned int startAgent();
	float	getEstimateDistance (Agent* solverAgent ,Agent* targetAgent);
	
	// Agent bewegen
	void	moveAgent(AgentState targetState);
	void	scanAgents();
	void	calculateFrame(Room* targetRoom);
	void	setID(int setID) {aID = setID;};
	void	pushToNotVisitedList (Agent* setTarget);
	void	setTarget(Agent* setTarget); 
	void	setCurrentState(AgentState setState);
	void	setNext (Room* setRoom);
	void	setnotvisitedAgents(vector <Agent *> setnotvisited);
	void	updateDistanceList();
	void	keepDistance();

	bool	initDistanceMap();
	bool	inFrame(Agent* targetAgent, Room* targetRoom, float targetFrame);
	bool	isSearchComplete();
	bool	planPath(Room * targetAgent);
	bool	alreadyTouched (Agent * contactAgent);
	bool	agentInVector (Agent * contactAgent, listAgents &searchVector); 

	Agent*	getClosestTarget(listAgents &oldTarget);
	Agent*	getClosestTarget(Agent* oldTarget);
	Agent*	getTarget(); 
	
	// get Raum
	Room*	getNext() { return nextRoom;}
	Room*	getCurrent() { return currentRoom;}
	// Pointer auf aktuellen Raum
	Room*	currentRoom;
	Room*	targetRoom;
	Room*	targetRoomOld;
	// Pointer auf nächsten Raum
	Room*	nextRoom;
	// set Room

	// Geplanter Weg
	listPath	plannedPath;
	listPathIterator plannedPathIterator;	
	
	// Vektor noch nicht besuchte Agenten
	visitedAgents	notvisitedAgents;
	visitedAgents	visitedAgentsList;
	visitedAgentsIterator findVisitedAgent (Agent * contactAgent);

	AgentState publicState;
protected:

private:

	int	aID;
	int	mWaitCount;
	int	mDistanceCount;
	bool	bsearchComplete;
	float	fsearchFrame;

	Agent *	targetAgent;
	AgentState currentState;
	AgentState nextState;
	
	mapDistance	distanceHeuristic;
	mapDistanceIterator distanceHeuristicIt, distanceHeuristicItUp;


};

#endif //_INCL_AGENT










