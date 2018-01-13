package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;
import com.sun.istack.internal.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * Object which represents a path created by the {@link IteratedLocalSearch}
 * algorithm.
 */
final class Route {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    private Graph graph;
    private Weighting weighting;
    private ShortestPathCalculator sp;
    private final int s, d;

    private List<Arc> arcs;
    private List<Path> blankSegments;
    private IntHashSet arcIds;
    private double cost, score;

    private Route(ShortestPathCalculator shortestPathCalculator, Graph graph, Weighting weighting, int s, int d) {
        sp = shortestPathCalculator;
        arcs = new ArrayList<>();
        blankSegments = new ArrayList<>();
        cost = 0;
        score = 0;
        this.s = s;
        this.d = d;
        this.graph = graph;
        this.weighting = weighting;
        arcIds = new IntHashSet();
    }

    public static Route newRoute(@NotNull ShortestPathCalculator sp, @NotNull Graph graph,
                                 @NotNull Weighting weighting, int s, int d) {
        return new Route(sp, graph, weighting, s, d);
    }

    void addArc(int index, @NotNull Arc arc) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        updatePathSegments(index, arc, arc);

        arcs.add(index, arc);
        cost += arc.cost;
        score += arc.score;
        arcIds.add(arc.edgeId);
    }

    public int removeArc(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        if(index == -1) {
            return index;
        }

        Path segment1 = blankSegments.remove(index);
        Path segment2 = blankSegments.remove(index);

        cost -= segment1.getDistance();
        cost -= segment2.getDistance();

        int length = getNumArcs();
        if(length > 1) {
            int start = s;
            int end = d;

            int prevIndex = index - 1;
            if(prevIndex >= 0 && prevIndex <= length - 1) {
                start = arcs.get(prevIndex).adjNode;
            }

            int nextIndex = index + 1;
            if(nextIndex <= length - 1) {
                end = arcs.get(nextIndex).baseNode;
            }
            Path segment = sp.shortestPath(start, end);
            blankSegments.add(index, segment);
            cost += segment.getDistance();
        }

        arcs.remove(index);
        cost -= a.cost;
        score -= a.score;
        arcIds.remove(a.edgeId);

        return index;
    }

    public void insertRoute(int index, @NotNull Route route) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        if(!route.isEmpty()) {
            Arc first = route.arcs.get(0);
            Arc last = route.arcs.get(route.getNumArcs() - 1);

            updatePathSegments(index, first, last);

            // We need to remove the inserted routes starting and ending path cost
            // We recalculate the new path segments below
            Path head = route.blankSegments.remove(0);
            Path tail = route.blankSegments.remove(route.blankSegments.size() - 1);
            route.cost -= head.getDistance();
            route.cost -= tail.getDistance();

            score += route.score;
            cost += route.cost;
            arcs.addAll(index, route.arcs);
            blankSegments.addAll(index + 1, route.blankSegments);
            arcIds.addAll(route.arcIds);
        }
    }

    private void updatePathSegments(int index, Arc left, Arc right) {
        int length = getNumArcs();
        int start = s, end = d;

        int startIndex = index - 1;
        if(startIndex >= 0 && startIndex <= length - 1) {
            start = arcs.get(startIndex).adjNode;
        }

        if(index <= length - 1) {
            end = arcs.get(index).baseNode;
        }

        Path segment1 = sp.shortestPath(start, left.baseNode);
        cost += segment1.getDistance();

        Path segment2 = sp.shortestPath(right.adjNode, end);
        cost += segment2.getDistance();

        if(length > 0) {
            Path removed = blankSegments.remove(index);
            cost -= removed.getDistance();
        }

        blankSegments.add(index, segment2);
        blankSegments.add(index, segment1);
    }


    public double getCost() {
        return cost;
    }

    public double getScore() {
        return score;
    }

    public Path getPath() {
        logger.debug("Route cost: " + getCost());
        Path path = new Path(graph, weighting);

        if(arcIds.contains(Arc.FAKE_ARC_ID)) {
            return path.setFound(false);
        }

        for(int i = 0; i < blankSegments.size(); i++) {
            Path blank = blankSegments.get(i);
            for(EdgeIteratorState edge : blank.calcEdges()) {
                path.processEdge(edge.getEdge(), edge.getAdjNode(), edge.getEdge());
            }

            if(i < arcs.size()) {
                Arc arc = arcs.get(i);
                path.processEdge(arc.edgeId, arc.adjNode, arc.edgeId);
            }
        }

        return path
                .setEndNode(d)
                .setFromNode(s)
                .setFound(!isEmpty());
    }

    private int getNumArcs() {
        return arcs.size();
    }

    public boolean isEmpty() {
        return getNumArcs() == 0;
    }

    public List<Arc> getArcs() {
        return arcs;
    }

    public List<Arc> getCandidateArcsByIP() {
        List<Arc> result = new ArrayList<>();
        double avgIP = 0;
        for(Arc ca : arcs) {
            ca.improvePotential = calcImprovePotential(ca);
            avgIP += ca.improvePotential;
        }
        avgIP /= arcs.size();

        for(Arc ca : arcs) {
            if(ca.improvePotential >= avgIP) {
                result.add(ca);
            }
        }

        return result;
    }

    private double calcImprovePotential(Arc arc) {
        int v1 = getPrev(arc).adjNode;
        int v2 = getNext(arc).baseNode;

        double score = 0;
        double maxDist = 0;

        double dist = sp.getPathCost(v1, v2, arc);

        for(Arc e : arc.getCas()) {
            score += e.score - arc.score;
            maxDist = Math.max(maxDist, sp.getPathCost(v1, v2, e));
        }

        double result = score / (maxDist - dist);

        // Hacky fix for NaN values
        return Double.isNaN(result) ? 0 : result;
    }

    public Arc getPrev(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index - 1 >= 0) ? arcs.get(index - 1) : a;
    }

    public Arc getNext(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index + 1 <= getNumArcs() - 1) ? arcs.get(index + 1) : a;
    }

    public Arc getArc(int index) {
        if(index >= 0 && index < getNumArcs()) {
            return arcs.get(index);
        }

        return null;
    }

    public boolean contains(@NotNull Arc a) {
        return arcIds.contains(a.edgeId);
    }

    public void insertArcAtMinPathSegment(@NotNull Arc arc, double budget) {
        int pathIndex = -1;
        double minPathValue = Double.MAX_VALUE;

        if(!isEmpty()) {
            // We have at least 1 arc and 2 blank path segments
            // Find smallest blank path segment
            for(int i = 0; i < blankSegments.size(); i++) {
                double value = blankSegments.get(i).getDistance();
                if(value < minPathValue) {
                    minPathValue = value;
                    pathIndex = i;
                }
            }

            int start = pathIndex == 0 ? s : arcs.get(pathIndex - 1).adjNode;
            int end = pathIndex == getNumArcs() ? d : arcs.get(pathIndex).baseNode;

            if(sp.getPathCost(start, end, arc) <=
                    budget - getCost() + minPathValue) {
                addArc(pathIndex, arc);
            }

        } else if(sp.getPathCost(s, d, arc) <= budget) {
            addArc(0, arc);
        }
    }
}
