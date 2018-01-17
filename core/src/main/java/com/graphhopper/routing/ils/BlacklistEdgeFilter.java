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
        return !blacklist.contains(edgeState.getEdge()) && edgeFilter.accept(edgeState);
    }

//    private boolean shortcutSkipsBlacklisted(EdgeIteratorState edgeIteratorState) {
//        if(edgeIteratorState == null) {
//            return false;
//        }
//
//        if(edgeIteratorState instanceof CHGraphImpl.CHEdgeIteratorImpl) {
//            CHGraphImpl.CHEdgeIteratorImpl chEdgeIterator = (CHGraphImpl.CHEdgeIteratorImpl) edgeIteratorState;
//
//            if(chEdgeIterator.isShortcut()) {
//                int edge1 = chEdgeIterator.getSkippedEdge1();
//                int edge2 = chEdgeIterator.getSkippedEdge2();
//
//                int to = chEdgeIterator.getAdjNode();
//
//                return blacklist.contains(chEdgeIterator.getEdge()) ||
//                        shortcutSkipsBlacklisted(routingGraph.getEdgeIteratorState(edge1, to)) ||
//                        shortcutSkipsBlacklisted(routingGraph.getEdgeIteratorState(edge2, to));
//
//            } else {
//                return blacklist.contains(chEdgeIterator.getEdge());
//            }
//        }
//
//        return blacklist.contains(edgeIteratorState.getEdge());
//    }
}
