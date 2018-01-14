package com.graphhopper.routing.ils;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Object which represents a path created by the {@link IteratedLocalSearch} algorithm.
 */
final class Route {
    List<Arc> edges;
    BitSet bitSet;
    double cost;
    double score;

    Route() {
        edges = new ArrayList<>();
        bitSet = new BitSet();
    }

    // Copy constructor
    private Route(Route route) {
        cost = route.cost;
        score = route.score;
        edges = new ArrayList<>(route.edges);
        bitSet = (BitSet) route.bitSet.clone();
    }


    void addEdge(int edgeId, int baseNode, int adjNode, double cost, double score) {
        edges.add(new Arc(edgeId, baseNode, adjNode, cost, score));
        bitSet.set(edgeId);
        this.cost += cost;
        this.score += score;
    }

    void removeEdge(int edgeId) {
        for(int i = edges.size() - 1; i >= 0; i--) {
            Arc arc = edges.get(i);
            if(arc.edgeId == edgeId) {
                edges.remove(i);
                bitSet.clear(edgeId);
                cost -= arc.cost;
                score -= arc.score;
                break;
            }
        }
    }

    Arc removeEdgeIndex(int index) {
        Arc arc = edges.remove(index);
        bitSet.clear(arc.edgeId);
        cost -= arc.cost;
        score -= arc.score;
        return arc;
    }

    void clear() {
        edges.clear();
        bitSet.clear();
        cost = 0;
        score = 0;
    }

    Route copy() {
        return new Route(this);
    }

    void splice(Route other, int index) {
        edges.addAll(index, other.edges);
        bitSet.or(other.bitSet);
        cost += other.cost;
        score += other.score;
    }

    void blacklist(Route other) {
        bitSet.or(other.bitSet);
    }

    @Override
    public String toString() {
        StringBuilder stringBuilder = new StringBuilder();
        for(Arc edge : edges) {
            stringBuilder.append("(")
                    .append(edge.baseNode)
                    .append(",")
                    .append(edge.adjNode)
                    .append(") ->");
        }
        return stringBuilder.toString();
    }

}
