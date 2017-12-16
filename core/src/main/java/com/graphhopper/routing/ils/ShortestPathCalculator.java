package com.graphhopper.routing.ils;

import com.graphhopper.routing.Path;

public interface ShortestPathCalculator {

    Path shortestPath(int s, int d);
}
