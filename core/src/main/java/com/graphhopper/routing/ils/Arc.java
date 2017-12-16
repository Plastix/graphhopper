package com.graphhopper.routing.ils;

import java.util.List;

/**
 * Class which contains metadata about a particular edge in the graph. Used by
 * {@link Route} and {@link IteratedLocalSearch}
 * <p>
 * In the ILS-CAS algorithm this represents an "attractive arc".
 */
final class Arc {
    final int edgeId, baseNode, adjNode;
    final double cost, score;
    double improvePotential, qualityRatio;
    private List<Arc> cas;

    Arc(int edgeId, int baseNode, int adjNode, double cost, double score) {
        this.edgeId = edgeId;
        this.baseNode = baseNode;
        this.adjNode = adjNode;
        this.cost = cost;
        this.score = score;
//        this.improvePotential = Double.MAX_VALUE;
    }

    @Override
    public String toString() {
        return "Arc{" +
                "edgeId=" + edgeId +
                '}';
    }

    public List<Arc> getCas() {
        return cas;
    }

    public void setCas(List<Arc> cas) {
        this.cas = cas;
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) return true;
        if(o == null || getClass() != o.getClass()) return false;

        Arc arc = (Arc) o;

        if(edgeId != arc.edgeId) return false;
        if(baseNode != arc.baseNode) return false;
        return adjNode == arc.adjNode;
    }

    @Override
    public int hashCode() {
        int result = edgeId;
        result = 31 * result + baseNode;
        result = 31 * result + adjNode;
        return result;
    }
}
