package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.PathBidirRef;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;

public class IlsPathCh extends PathBidirRef {

    private IntHashSet edges;
    private Graph routingGraph;

    IlsPathCh(Graph g, Weighting weighting) {
        super(g, weighting);
        this.routingGraph = g;
        this.edges = new IntHashSet();
    }

    @Override
    public void processEdge(int tmpEdge, int endNode, int prevEdgeId) {
        super.processEdge(tmpEdge, endNode, prevEdgeId);
        edges.add(routingGraph.getEdgeIteratorState(tmpEdge, endNode).getEdge());
    }

    public IntHashSet getEdges() {
        return edges;
    }
}
