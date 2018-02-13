package com.graphhopper.routing.ils;

import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.util.EdgeIteratorState;

/**
 * Weighting classed use to calculate scores of roads for the {@link IteratedLocalSearch} routing algorithm.
 */
public class NormalizedBikePriorityWeighting extends BikePriorityWeighting {

    private double cutoff;

    NormalizedBikePriorityWeighting(FlagEncoder encoder, double cutoff) {
        super(encoder);
        this.cutoff = cutoff;
    }

    @Override
    public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
        double weight = super.calcWeight(edgeState, reverse, prevOrNextEdgeId);
        return weight > cutoff ? 1 : 0;
    }
}
