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

    BlacklistEdgeFilter(Graph graph, @NotNull EdgeFilter edgeFilter, @NotNull IntHashSet blacklist) {
        routingGraph = graph;
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
                int from = chEdge.getBaseNode();

                return blacklist.contains(chEdge.getEdge()) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge1, to)) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge2, to)) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge1, from)) ||
                        isBlacklisted(routingGraph.getEdgeIteratorState(edge2, from));
            }
        }

        return blacklist.contains(edgeIteratorState.getEdge());
    }


}
