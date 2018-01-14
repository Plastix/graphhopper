package com.graphhopper.routing.ils;

/**
 * Class which contains metadata about a particular edge in the graph. Used by
 * {@link Route} and {@link IteratedLocalSearch}
 */
class Arc {
    final int edgeId, baseNode, adjNode;
    final double cost, score;

    Arc(int edgeId, int baseNode, int adjNode, double cost, double score) {
        this.edgeId = edgeId;
        this.baseNode = baseNode;
        this.adjNode = adjNode;
        this.cost = cost;
        this.score = score;
    }

    @Override
    public String toString() {
        return "Arc{" +
                "edgeId=" + edgeId +
                '}';
    }
}
