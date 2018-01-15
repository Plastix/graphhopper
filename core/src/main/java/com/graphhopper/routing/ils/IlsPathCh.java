package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.ch.Path4CH;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;

public class IlsPathCh extends Path4CH {

    private IntHashSet edges;
    private Graph routingGraph;

    IlsPathCh(Graph routingGraph, Graph baseGraph, Weighting weighting) {
        super(routingGraph, baseGraph, weighting);
        this.routingGraph = routingGraph;
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
