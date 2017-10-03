package com.graphhopper.routing.weighting;

import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.PriorityCode;
import com.graphhopper.util.EdgeIteratorState;

import static com.graphhopper.routing.weighting.PriorityWeighting.KEY;

// TODO (Aidan) 
public class BikePriorityWeighting extends AbstractWeighting {

    public BikePriorityWeighting(FlagEncoder encoder) {
        super(encoder);
    }

    @Override
    public double getMinWeight(double distance) {
        return PriorityCode.WORST.getValue();
    }

    @Override
    public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
        return flagEncoder.getDouble(edgeState.getFlags(), KEY);
    }

    @Override
    public String getName() {
        return "bike priority";
    }
}
