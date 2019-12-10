/*
 *  Timer.hpp
 *
 *  Created by Andrin Jenal
 *  Copyright 2013 ETH Zurich. All rights reserved.
 * 
 */

#include <ctime>

class Timer
{
    std::time_t t_start, t_end; 
	
public:
	
	void start()
	{
        t_start = std::time(NULL);
	}
	
	double stop()
	{
        t_end = std::time(NULL);
        return (t_end - t_start);
	}
};
