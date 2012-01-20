/**********************************************************************\
* Dateiname: maze_mario_grotschar.cpp
* Autor : Mario Grotschar
*		  Gerardo Martinez
*		  Christoph Eder 
*
* Projekt : Projekt 3 Die Besucher
* Copyright (C) <<COPYRIGHT>>
*
** Kurzbeschreibung: Beinhaltet die Mainfunktion, sowie Funktionen zum
*					 Erzeugen und Ausduennen des Maze sowie
*					zur Zeitmessung des Projekts.
*
* Datum: Autor: Grund der Aenderung:
*
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/

// -----------------------------------------------------------------
// The algorithm for creating random mazes works as follows:
//
//    - Suppose we have 5 rows and 8 columns ...
// 
//       +-+-+-+-+-+-+-+-+
//       | | | | | | | | |
//       +-+-+-+-+-+-+-+-+
//       | | | | | | | | |
//       +-+-+-+-+-+-+-+-+
//       | | | | | | | | |
//       +-+-+-+-+-+-+-+-+
//       | | | | | | | | |
//       +-+-+-+-+-+-+-+-+
//       | | | | | | | | |
//       +-+-+-+-+-+-+-+-+
//
//     The perimeter of the maze will always remain in place
//     but we'll construct the maze by knocking down a number
//     of the interior walls.
//
//    - So, in this example, for each row, we have 7 (8 - 1)
//      interior walls => 5 rows * 7 per row => 35 vertical interior walls
//
//      We also have 8 horizontal interior walls for each row
//      (except the last one) => 4 (5 - 1) rows * 8 per row => 32
//      horizontal interior walls
//
//      For 'R' rows and 'C' columns we'll have:
//         (R - 1) * C + (C - 1) * R -> total interior walls
//
//    - We then put all our walls into an array and randomize
//      them.  Afterwards, we start at the beginning of the array
//      and work our way through all the interior walls knocking
//      down any walls that would connect rooms not already
//      connected (reachable from each other currently)
//
//    - The way we determine if two rooms are connected is as follows:
//
//      - A group is a list of rooms
//
//      - We start with as many groups as there are rooms and
//        each group containing only one room.  This reflects
//        the initial state of the maze with no rooms being
//        connected to any other rooms
//
//      - As we consider an interior wall, we see which group
//        the two rooms it separates belongs to.  If the rooms are
//        in the same group we leave the wall in place.  If the rooms
//        are in different groups we knock down the wall and absorb
//        all members of the group room two belongs to into the
//        group room one belongs to.
//
//    - Finally, we also determine the longest possible path
//      through the maze and use that path to determine the maze's
//      start and end points.  We find the longest path by:
//
//      - start with an empty list of the rooms we've already visited
//
//      - we begin with room(0, 0)
//
//      - each time we visit a room, we'll add that room to the list
//        of rooms we've already visited
//
//      - for each room, we'll examine if we have neighbors (rooms)
//        to the north, south, east and west and recursively visit
//        them if we've not already been to that room
//
//      - eventually, we'll run out of rooms to visit at which point
//        we report back where we ended up and how many hops it took
//        us to get there, we record the ending room that had the
//        largest number of hops
//
//      - using the above, we find the longest path from room(0, 0), but
//        this may not be the longest path through the maze overall, we
//        find that longest path by repeating the above steps but using
//        as the starting room, the room that was farthest from room(0, 0)
//
// Usage:
//
//    maze { -size rowsXcolumns } { -ps }
//       -size 50x75 => 50 rows by 75 columns (default is 16x16)
//       -h          => display help information
// -----------------------------------------------------------------

#define THEWIDTH 1
#define DEBUG_LISTS 0
#define DEBUG	1
#define DISPLAY	1
/*****************************************************************
 * C/C++ Headers
 *****************************************************************/

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
#include <windows.h> 

#include "pdc34dllw/curses.h"
#include "stlastar.h" // See header for copyright and usage information
#include "maze.h"
//#include "planner.h" Already included in agent.
#include "agent.h"


using  std::string;
using  std::vector;

/*****************************************************************
 * Local Defines
 *****************************************************************/


#define N_ROWS    (20)
#define N_COLUMNS (20)

const static int static_AgentNumbers = 10;


/*****************************************************************
 * Local Types
 *****************************************************************/

// In maze.h

/*****************************************************************
 * Local Function Prototypes
 *****************************************************************/

static void
seedRandom ();

static string
toString (long value);

static void
getLongestPath (Rooms & rooms, Room ** beginPtr, Room ** endPtr);

static void
dumpTxt (Rooms & rooms);

static void
init (int rows,
      int columns,
      Rooms & rooms,
      Walls & walls,
      Cleaner < Room > &roomCleaner, Cleaner < Wall > &wallCleaner);

static bool
equalsIC (char *first, char *second);

static bool
getArgs (int argc,
	 char **argv,
	 int *rowsPtr,
	 int *columnsPtr,
	 bool * helpPtr, string & errMsg);

/*****************************************************************
 * Local Type Definitions
 *****************************************************************/


/*****************************************************************
 * Local Functions
 *****************************************************************/

static void seedRandom ()
{
  srand ((unsigned int)time (NULL));
}

static string toString (long value)
{
  char buffer[255] = { '\0' };

  sprintf_s (buffer, 255, "%ld", value);

  string s = buffer;

  return (s);
}

static void
getLongestPath (Rooms & rooms, Room ** beginPtr, Room ** endPtr)
{
  // First, find the room that requires the longest path to
  // reach from the cell with row = 0, column = 0.

  Room *theRoom = rooms[0][0];
  Group history;

  theRoom->visit (0, history, theRoom);

  Room *rFirst = theRoom->getMaxRoom ();

  // Now, find the room that requires the longest path to
  // reach from 'rFirst'.  We need to clear our history
  // so we'll be allowed to visit our neighbors again.

  history.clear ();

  rFirst->visit (0, history, rFirst);

  Room *rLast = rFirst->getMaxRoom ();

  *beginPtr = rFirst;
  *endPtr = rLast;
}


static void
dumpTxt (Rooms & rooms)
{
  int nRows = rooms.size ();
  int nColumns = rooms[0].size ();

  string s =
    "Rows: " + toString (nRows) + ", Columns: " + toString (nColumns);

  bool hasFirstAndLast = false;
   hasFirstAndLast = true;


  printw ("========================================================\n"
	  "%s\n"
	  "========================================================\n",
	  s.c_str ());

  const char *indent = "   ";
  int theWidth = THEWIDTH;
  string dashes = "";
  string spaces = "";
  int i = 0;
  int j = 0;

  for (i = 0; i < theWidth; i++)
    {
      dashes += "-";
      spaces += " ";
    }

  for (i = nRows - 1; i >= 0; i--)
    {
      if (i == nRows - 1)
	{
	  printw ("%s+", indent);

	  for (j = 0; j < nColumns; j++)
	    printw ("%s+", dashes.c_str ());

	  printw ("\n");
	}

      for (j = 0; j < nColumns; j++)
	{
	  if (j == 0)
	    printw ("%s|", indent);

	  char c = ' ';

	  if (hasFirstAndLast)
	    {
	      if (rooms[i][j]->occupiedRobot)
			  c =  0x30 +  rooms[i][j]->occupiedRobot->getID() ;

	    }

	  printw ("%c%c", c, rooms[i][j]->hasEast ()? ' ' : '|');
	}

      printw ("\n");
      printw ("%s+", indent);

      for (j = 0; j < nColumns; j++)
	{
	  if (i == 0 || !rooms[i][j]->hasSouth ())
	    printw ("%s+", dashes.c_str ());

	  else
	    printw ("%s+", spaces.c_str ());
	}

      printw ("\n");
    }
}

static void
init (int rows,
      int columns,
      Rooms & rooms,
      Walls & walls,
      Cleaner < Room > &roomCleaner, Cleaner < Wall > &wallCleaner)
{
  int i = 0;
  int j = 0;
  int id = 0;

  for (i = 0; i < rows; i++)
    {
      RoomRow row;

      for (int j = 0; j < columns; j++, id++)
	{
	  Room *roomPtr = new Room (id, i, j);

	  row.push_back (roomPtr);
	  roomCleaner.add (roomPtr);
	}

      rooms.push_back (row);
    }

  for (i = 0; i < rows; i++)
    {
      for (j = 1; j < columns; j++)
	{
	  Room *first = rooms[i][j - 1];
	  Room *second = rooms[i][j];
	  Wall *wallPtr = new Wall (first, second);

	  walls.push_back (wallPtr);
	  wallCleaner.add (wallPtr);
	}
    }

  for (i = 1; i < rows; i++)
    {
      for (j = 0; j < columns; j++)
	{
	  Room *first = rooms[i - 1][j];
	  Room *second = rooms[i][j];
	  Wall *wallPtr = new Wall (first, second);

	  walls.push_back (wallPtr);
	  wallCleaner.add (wallPtr);
	}
    }
}

static bool
equalsIC (char *first, char *second)
{
  int nFirst = strlen (first);
  int nSecond = strlen (second);

  if (nFirst != nSecond)
    return (false);

  while (*first)
    {
      if (toupper (*first) != toupper (*second))
	return (false);

      first++;
      second++;
    }

  return (true);
}


static bool
getArgs (int argc,
	 char **argv,
	 int *rowsPtr,
	 int *columnsPtr,
	 bool * helpPtr, string & errMsg)
{
  *rowsPtr = N_ROWS;
  *columnsPtr = N_COLUMNS;
  *helpPtr = false;

  for (int i = 1; i < argc; i++)
    {
      if (argv[i][0] == '-')
	{
	  char *ptr = argv[i] + 1;

	  if (equalsIC (ptr, "size"))
	    {
	      if (i + 1 == argc)
		{
		  errMsg = "Missing value after '-size'";
		  return (false);
		}

	      ptr = argv[i + 1];
	      string s = ptr;
	      char *x = NULL;

	      if ((x = strchr (ptr, 'x')) == NULL &&
		  (x = strchr (ptr, 'X')) == NULL)
		{
		  errMsg = "Invalid size: [" + s + "]";
		  return (false);
		}

	      char *endPtr = NULL;
	      long int rows = strtol (ptr, &endPtr, 10);

	      if (endPtr != x)
		{
		  errMsg = "Invalid size: [" + s + "]";
		  return (false);
		}

	      long int columns = strtol (x + 1, &endPtr, 10);

	      if (*endPtr)
		{
		  errMsg = "Invalid size: [" + s + "]";
		  return (false);
		}

	      if (rows <= 1 || columns <= 1)
		{
		  errMsg = "Rows and columns must be at least 2: [" + s + "]";
		  return (false);
		}

	      *rowsPtr = rows;
	      *columnsPtr = columns;
	    }
	  else if (equalsIC (ptr, "h"))
	    {
	      *helpPtr = true;
	    }
	  else
	    {
	      errMsg = "Invalid argument: [";
	      errMsg += argv[i];
	      errMsg += "]";

	      return (false);
	    }
	}
    }

  return (true);
}

static Room* getRandomRoom( Rooms _rooms)
{
	Room*	tempRoom;
	
	int theRow = rand ()%N_ROWS;
	int theColumn = rand ()%N_COLUMNS;

	tempRoom = _rooms[theRow][theColumn];

	return tempRoom;

}

static void	initAgents(int agentNumbers, vector <Agent> &setAgents, Rooms _rooms)
{
	Agent tempAgent;
	vector <Agent *> tempSetAgents;
	bool duplicate;

	for (int loop = 0; loop < agentNumbers; loop++)
	{
	  do 
	  {
		duplicate = 0;		
		  tempAgent.currentRoom = getRandomRoom(_rooms);
		// check if room is alread occupied
		for (unsigned int i=0; i<setAgents.size();i++)
			{
			if (setAgents[i].currentRoom == tempAgent.currentRoom)
				{
				duplicate = 1;
				cout << "gewaehlter Raum (random) bereits besetzt -> neuer Raum wird gewaehlt" << endl;
				}
			}
	   } while (duplicate == 1);

	  tempAgent.nextRoom = NULL;
	  tempAgent.setID(loop);
	 
	  setAgents.push_back(tempAgent);

	  #ifdef DEBUG
	  printf ("Room Nr.: %d mID: %d x: %d y: %d\n", loop, tempAgent.getCurrent()->mID, tempAgent.getCurrent()->mColumn, tempAgent.getCurrent()->mRow); 
	  #endif
	}

	// Create vector of agent pointers as setnotvisitedAgents needs this type (save memory).
	for (int loop = 0; loop < agentNumbers; loop++)
	{
		tempSetAgents.push_back(&setAgents[loop]);
	}
	// Save the generated vector in each agent (including himself). The pointer to agent himself can be deleted in the init state.
	for (int loop = 0; loop < agentNumbers; loop++)
	{
		setAgents[loop].setnotvisitedAgents(tempSetAgents);
	}
}

double static diffclock (clock_t start, clock_t finish)
{
		double diffticks= (double)finish- (double)start;
		double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
		return diffms;
}

void static color (int c)
{
if (c < 8) {
         attron(COLOR_PAIR(c));
         attroff(A_BOLD);
     }
     else if (c == 8) {
         attron(COLOR_PAIR(0) | A_BOLD);
     }
     else {
         attron(COLOR_PAIR(c - 8) | A_BOLD);
     }
}

/*****************************************************************
 * main
 *****************************************************************/

int main (int argc, char **argv)
{
  int rows = N_ROWS;
  int columns = N_COLUMNS;
  int tornWalls = 0;
  bool help = false;
  string errMsg = "";
  clock_t begin_search ,end_search;
  vector <Agent> agentList;

	/* PDcurses functions*/
	/* Initialize the screen */
	initscr(); 
	raw(); // line-buffering off, echo off, etc.
	nonl();

	/* Init color map */
	start_color();
	init_pair(0, COLOR_BLACK, COLOR_BLACK);
	init_pair(1, COLOR_BLUE, COLOR_BLACK);
	init_pair(2, COLOR_GREEN, COLOR_BLACK);
	init_pair(3, COLOR_CYAN, COLOR_BLACK);
	init_pair(4, COLOR_RED, COLOR_BLACK);
	init_pair(5, COLOR_MAGENTA, COLOR_BLACK);
	init_pair(6, COLOR_YELLOW, COLOR_BLACK);
	init_pair(7, COLOR_WHITE, COLOR_BLACK);

 


  bool status =
    getArgs (argc, argv, &rows, &columns, &help, errMsg);

  if (!status || help)
    {
      printf ("Usage: %s { -ps } { -size rowsXcolumns }\n"
	      "   -size 50x75 => 50 rows by 75 columns\n"
	      "   -h          => display this message\n", argv[0]);

      if (!status)
	printf ("\n%s\n", errMsg.c_str ());

      return (1);
    }

  if (false)
    {
      printf ("Rows: [%d], Columns: [%d]\n",
	      rows, columns);
    }

  Rooms rooms;
  Walls walls;

  Cleaner < Room > roomCleaner;
  Cleaner < Wall > wallCleaner;

  init (rows, columns, rooms, walls, roomCleaner, wallCleaner);

  int nWalls = walls.size (); // Holds the number of inner walls. First halft of vector refers to vertical walls secound half to horizontal walls.

  // ------------------------------------------------
  // Shuffle the walls
  // ------------------------------------------------

  seedRandom ();

  int i = 0;
  int j = 0;

  for (i = 0; i < nWalls; i++)
    {
      int idx1 = 0;
      int idx2 = 0;

      do
	{

	  int value1 = rand ();
	  int value2 = rand ();

	  // Use absolute value of value1 / value2
	  if (value1 < 0)
	    value1 = -value1;
	  if (value2 < 0) 
		value2 = -value2;

	  idx1 = value1 % nWalls;
	  idx2 = value2 % nWalls;

	}
      while (idx1 == idx2);
	  
	  // Put some vertical walls to  first half and some of the horizontal to the second half.
      Wall *ptr = walls[idx1];
      walls[idx1] = walls[idx2];
      walls[idx2] = ptr;
    }

  // ---------------------------------------------
  // Put each room initially into its own group
  // ---------------------------------------------

  GroupIdToGroup gMap;

  for (i = 0; i < rows; i++)
    {
      for (j = 0; j < columns; j++)
	{
	  Room *r = rooms[i][j];

	  Group theGroup;

	  theGroup.insert (r->mID);

	  gMap[r->mID] = theGroup;
	  r->setGroupID (r->mID);
	}
    }

  // ------------------------------------------------
  // Now start knocking down walls ...
  // ------------------------------------------------

  for (i = 0; i < nWalls; i++)
    {
      if (false)
	{
	  printw ("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	  printw ("(%d / %d)\n", i + 1, nWalls);
	  printw ("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	  dumpTxt (rooms);
	}

      Room *first = walls[i]->getFirst ();
      Room *second = walls[i]->getSecond ();

      // Are these two rooms already reachable
      // from one another?

      if (first->getGroupID () != second->getGroupID ())
	{
	  // Not yet reachable so correct that ...

	  if (first->mColumn == second->mColumn)
	    {
		  // Tear down horizontal wall
	      first->setNorth (second);
	      second->setSouth (first);
		  tornWalls++;  
	  }
	  else
	    {
		  // Tear down vertical wall
	      first->setEast (second);
	      second->setWest (first);
		  tornWalls++;
	    }

	  // Now, we need to put everyone from second's group
	  // into first's group

	  int sid = second->getGroupID ();
	  Group theGroup = gMap[sid];

	  Group::iterator gi;

	  for (gi = theGroup.begin (); gi != theGroup.end (); gi++)
	    {
	      int roomID = *gi;
	      int theRow = roomID / columns;
	      int theColumn = roomID % columns;

	      Room *ptr = rooms[theRow][theColumn];
	      int gid = first->getGroupID ();

	      gMap[gid].insert (roomID);
	      ptr->setGroupID (gid);
	    }

	  gMap.erase (sid);
	}
    }
  
  // Labyrinth ausdünnen
  // Übriggebliebene Wände um 50% dezimieren.
  #ifdef DEBUG
	printf ("Initially torn down walls: %d\n ", tornWalls);
#endif

  int leftWalls = nWalls - tornWalls; // Number of left walls
  tornWalls = 0;

  do
  {
	  int value1 = rand ();
	  value1 = abs(value1);
	  
	  // Get a random wall
	  value1 %= (walls.size()-1);
	  Room *first = walls[value1]->getFirst ();
      Room *second = walls[value1]->getSecond ();

	if (first->mColumn == second->mColumn)
	{
	// Tear down horizontal wall
		if (!(walls[value1]->getFirst()->hasNorth()) && !(walls[value1]->getSecond()->hasSouth()))
		{
	      first->setNorth (second);
	      second->setSouth (first);
		  tornWalls++; 
		}
	  }
	  else
	  {
		if (!(walls[value1]->getFirst()->hasEast()) && !(walls[value1]->getSecond()->hasWest()))
		{
		  // Tear down vertical wall
	      first->setEast (second);
	      second->setWest (first);
		  tornWalls++;
		}
	  }


  } while (tornWalls < (leftWalls/2));
 

#ifdef DEBUG
	printf ("Torn down walls: %d ", tornWalls+(nWalls-leftWalls));
#endif



  initAgents(static_AgentNumbers, agentList, rooms);

	vector<int> tempRes;
	int iteratNR = 0;
	int completeCounter = 0;

	cout << "Press any key to start.\n";
	getchar();

	begin_search = clock();

  
	do
	{
		//Sleep(500);
		refresh();
		//Sleep(100);
		clear();

		// Counterreset.
		completeCounter = 0;
		// Calculation part.
		for (int ag_n = 0; ag_n < static_AgentNumbers; ag_n ++)
		{
			agentList[ag_n].startAgent();
			if (agentList[ag_n].isSearchComplete())
				++completeCounter;

		}

		
		#if DISPLAY
		// Output part
		void (*dump) (Rooms &) = dumpTxt;
		dump (rooms);


		if (static_AgentNumbers <= 10)
		for (unsigned int i= 0; i < agentList.size(); i++)
		{
			
			printw ("\nAgNr %d Tar: %d:  ",agentList[i].getID(), agentList[i].getTarget()->getID()); 
			for (unsigned int j= 0; j < agentList[i].visitedAgentsList.size(); j++)
			{	
				color(2);
				printw ("V-%d |", agentList[i].visitedAgentsList[j]->getID());
			}
			for (unsigned int z= 0; z < agentList[i].notvisitedAgents.size(); z++)
			{	
				color(4);
				printw ("n-%d |", agentList[i].notvisitedAgents[z]->getID());
			}

			color(7);
		}
		#endif

		#if !DISPLAY
			printw ("\n %d out of %d completed.", completeCounter, static_AgentNumbers);
		#endif
		
		iteratNR++;
		if (completeCounter == static_AgentNumbers)
			break;

		 /* Because curses stores your window data in a structure; you need to refresh() when you want
                * to see what you've done */
	} while(true);
	end_search = clock();
	refresh();

	color (4);
	printw ("\nSearch is complete. It took %f ms.",double(diffclock(begin_search,end_search)));
    refresh();

    endwin(); /* Destroy the curses window */
  return (0);
}
