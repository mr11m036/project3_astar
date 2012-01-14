/**********************************************************************\
* Dateiname: maze.h
* Autor : Mario Grotschar
*		  Gerardo Martinez
*		  Christoph Eder 
* Projekt : Projekt 3 Die Besucher
* Copyright (C) <<COPYRIGHT>>
*
* Kurzbeschreibung: headerfile fuer maze.cpp
*
* Datum: Autor: Grund der Aenderung:
* 7.12.2011 Mario Grotschar Neuerstellung
* <<DATUM>> <<AUTOR>> <<AENDERUNGSGRUND>>
*
\**********************************************************************/
#ifndef _INCL_MAZE
#define _INCL_MAZE
#include <iostream>


/*****************************************************************
 * Local Types
 *****************************************************************/
/*--- #define-Konstanten und Makros ----------------------------------*/

/*--- Datentypen (typedef) -------------------------------------------*/
typedef std::set < int >  Group;
typedef std::map < int,  Group >  GroupIdToGroup;

using namespace std;

class
  Room
{
	friend class MapSearchNode;
	friend class Agent;

public:
  Room (int id, int row, int column);

  void  visit (int depth, Group & visited, Room * caller);
  void  deadEndFound (int depth, Room * room);
  Room *  getMaxRoom ();

  bool	df_search (Group & visited, Room * exitRoom);
  bool  bf_search (Group & visited, Room * startRoom, Room * exitRoom);
  bool	id_search (Group & visited, Room * exitRoom, int idDeph);


  bool  hasNorth ();
  bool  hasSouth ();
  bool  hasEast ();
  bool  hasWest ();
  Room*  getNorth () {return mNorth;}
  Room*  getSouth (){return mSouth;}
  Room*  getEast () {return mEast;}
  Room*  getWest ( ){return mWest;}
  bool  isFree ();

  void  setNorth (Room * north);
  void  setSouth (Room * south);
  void  setEast (Room * east);
  void  setWest (Room * west);

  int	getCost ();
  // Parent setter and getter
  void setParent (Room * parent);
  Room * getParent ();

  void  setGroupID (int id);
  int  getGroupID ();

  bool getIdsFlag();



public:
  const int    mID;
  const int    mRow;
  const int    mColumn;
  const int	   mCost;

  int	mValue;
  bool	mFlag;
  bool	mFlagBFS;
  bool	mFlagIDS;
  bool	mFlagAstar;

  Agent*	occupiedRobot;

private:
  int    mGroupID;

  Room *    mNorth;
  Room *    mSouth;
  Room *    mEast;
  Room *    mWest;

  Room * mParent;

  Room *    mMaxRoom;
  int    mMaxDepth;
};
typedef
  vector <Room * >  RoomRow;
typedef
  vector <  RoomRow >  Rooms;

class
  Wall
{
public:
  Wall (Room * first, Room * second);
  Room *  getFirst ();
  Room *  getSecond ();

private:
  Room *    mFirst;
  Room *    mSecond;
  bool    mPresent;
};

typedef
  vector <Wall * >  Walls;


template < typename T > class Cleaner
{
public:
  Cleaner ();
  ~Cleaner ();
  void
  add (T * ptr);

private:
  Cleaner (const Cleaner &);
  Cleaner & operator= (const Cleaner &);

private:
  vector < T * >mList;
};

template < typename T > Cleaner < T >::Cleaner ()
{
}

template < typename T > Cleaner < T >::~Cleaner ()
{
  for (int n = mList.size (), i = 0; i < n; i++)
    {
      delete
	mList[i];
      mList[i] = NULL;
    }

  mList.clear ();
}

template < typename T > void
  Cleaner <
T >::add (T * ptr)
{
  mList.push_back (ptr);
}


#endif
