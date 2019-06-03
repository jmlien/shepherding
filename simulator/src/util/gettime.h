//------------------------------------------------------------------------------
//  Copyright 2010-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#pragma once
#ifndef _GETTIME_H_
#define _GETTIME_H_

//-----------------------------------------------------------------------------

#ifdef _WIN32
#include <windows.h>
#include <Mmsystem.h>
#else
#include <sys/time.h>
#endif

//------------------------------------------------------------------------
inline double getTime() //in millisecond
{
#ifdef WIN32
    return timeGetTime();
#else 
    //assuming unix-type systems
    //timezone tz;
    timeval  tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000000+tv.tv_usec)*1.0/1000;
#endif
}

#endif //_GETTIME_H_

