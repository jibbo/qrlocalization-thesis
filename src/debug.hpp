/* 
 * File:   debug.h
 * Author: j
 *
 * Created on May 31, 2012, 1:32 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef DEBUG
#define DBGVAR(X) std::cout << "var " #X " : " <<std::endl<< (X)<<std::endl
#define PDEBUG(X) std::cout << (X) << std::endl
#define NDEBUG(X) std::cout << "var" #X " : " << std::endl
#else
#define PDEBUG(X)
#define DBGVAR(X) 
#define NDEBUG(X)
#endif

#endif	/* DEBUG_H */

