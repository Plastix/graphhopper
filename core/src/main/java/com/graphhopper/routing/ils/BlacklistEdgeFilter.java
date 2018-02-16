package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.cursors.IntCursor;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.CHGraphImpl;
import com.graphhopper.util.EdgeIteratorState;
import com.sun.istack.internal.NotNull;

/**
 * EdgeFilter which blacklists a set EdgeIds. If an edge is not in the blacklist it defaults to the passed
 * in EdgeFilter.
 */
public class BlacklistEdgeFilter implements EdgeFilter {

    private EdgeFilter edgeFilter;
    private IntHashSet blacklist;

    BlacklistEdgeFilter(@NotNull EdgeFilter edgeFilter, @NotNull IntHashSet blacklist) {
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

            for(IntCursor intCursor : chEdge.getSkippedEdges()) {
                if(!blacklist.contains(intCursor.value)) {
                    return false;
                }
            }
        }

        return blacklist.contains(edgeIteratorState.getEdge());
    }
}
