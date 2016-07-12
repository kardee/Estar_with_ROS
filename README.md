# Estar_with_ROS

Estar Library taken from https://github.com/poftwaresatent/estar2
Copyright (C) 2013, Roland Philippsen. All rights reserved. Released under a 3-clause BSD license.

I have created the ROS wrapper for the estar library and called necessary functions from the library in the following order.

1. estar_init();
2. estar_set_speed(); 
3. estar_set_goal();
4. estar_propogate();

And for the trace back, I have copied the code from gestar.c(which I have not added in this repository) and added the few lines of code for pose estimate (219-228 lines).

