package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.CHGraphImpl;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;
import com.sun.istack.internal.NotNull;

/**
 * EdgeFilter which blacklists a set EdgeIds. If an edge is not in the blacklist it defaults to the passed
 * in EdgeFilter.
 */
public class BlacklistEdgeFilter implements EdgeFilter {

    private Graph routingGraph;
    private EdgeFilter edgeFilter;
    private IntHashSet blacklist;

    BlacklistEdgeFilter(Graph routingGraph, @NotNull EdgeFilter edgeFilter, @NotNull IntHashSet blacklist) {
        this.routingGraph = routingGraph;
        this.edgeFilter = edgeFilter;
        this.blacklist = blacklist;
    }

    @Override
    public boolean accept(EdgeIteratorState edgeState) {
        return !isBlacklisted(edgeState) && edgeFilter.accept(edgeState);
    }

    private boolean isBlacklisted(EdgeIteratorState edgeIteratorState) {
        if(edgeIteratorState == null) {
            return false;
        }

        if(edgeIteratorState instanceof CHGraphImpl.CHEdgeIteratorImpl) {
            CHGraphImpl.CHEdgeIteratorImpl chEdge = (CHGraphImpl.CHEdgeIteratorImpl) edgeIteratorState;

            if(chEdge.isShortcut()) {
                int edge1 = chEdge.getSkippedEdge1();
                int edge2 = chEdge.getSkippedEdge2();
                int to = chEdge.getAdjNode();

                return blacklist.contains(chEdge.getEdge()) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge1, to)) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge2, to));
            }
        }

        return blacklist.contains(edgeIteratorState.getEdge());
    }
}
