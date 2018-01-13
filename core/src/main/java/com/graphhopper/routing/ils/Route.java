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
    private final int s, d; // Start & End Node IDs

    private List<Arc> arcs; // List of "attractive arcs" in the Route
    private List<Path> blankSegments; // List of shortest paths connecting non-contiguous attractive arcs.
    private IntHashSet arcIds; // Hashset of Arc IDs from arcs
    private double cost, score; // Current

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

    /**
     * Static factory method for creating a new Route instance.
     *
     * @param sp        Interface which can calculate Shortest Paths.
     * @param graph     Graph.
     * @param weighting Weighting used to calculate distance of final path.
     * @param s         Start Node ID.
     * @param d         End Node ID.
     * @return New Route Instance.
     */
    static Route newRoute(@NotNull ShortestPathCalculator sp, @NotNull Graph graph,
                          @NotNull Weighting weighting, int s, int d) {
        return new Route(sp, graph, weighting, s, d);
    }

    /**
     * Adds the specified Arc to the Route at the specified index.
     * Throws {@link IndexOutOfBoundsException} if index <= 0 or index > {@link Route#getNumArcs()}.
     *
     * @param index Index to insert Arc.
     * @param arc   Arc to insert.
     */
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

    /**
     * Removes the first instance of the specified Arc from the Route.
     *
     * @param a Arc to remove.
     * @return Index of removed Arc. Returns -1 if Arc was not in the current Route.
     */
    int removeArc(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        // Short circuit if Arc is not present in Route
        if(index == -1) {
            return index;
        }

        // Remove two path segments surrounding Arc
        Path segment1 = blankSegments.remove(index);
        Path segment2 = blankSegments.remove(index);
        cost -= segment1.getDistance();
        cost -= segment2.getDistance();

        // If we have more than 1 arc we need to add a new path segment to join the Route
        int length = getNumArcs();
        if(length > 1) {
            int start = s;
            int end = d;

            // Calculate start/end points for the new blank path segment
            int prevIndex = index - 1;
            if(prevIndex >= 0 && prevIndex <= length - 1) {
                start = arcs.get(prevIndex).adjNode;
            }

            int nextIndex = index + 1;
            if(nextIndex <= length - 1) {
                end = arcs.get(nextIndex).baseNode;
            }

            // Calculate and add new path segment
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

    /**
     * Adds the specified Route to the current Route at the specified index.
     * Throws {@link IndexOutOfBoundsException} if index <= 0 or index > {@link Route#getNumArcs()}.
     *
     * @param index Index to insert Route.
     * @param route Route to insert.
     */
    void insertRoute(int index, @NotNull Route route) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        // Only add Route if it is non-empty
        if(!route.isEmpty()) {
            Arc first = route.arcs.get(0);
            Arc last = route.arcs.get(route.getNumArcs() - 1);

            updatePathSegments(index, first, last);

            // We need to remove the inserted routes starting and ending path segments
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

    /**
     * Updates the blank path segments at the specified index. Used when adding a new Arc to the route.
     * <p>
     * 1-2 --> 1-3-2
     *
     * @param index Index of blank path segments to update.
     * @param left  Left bound of the Arc to be inserted.
     * @param right Right bound of the Arc to be inserted.
     */
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

        // If non-empty, remove the previous blank path segment before inserting the two new ones
        if(length > 0) {
            Path removed = blankSegments.remove(index);
            cost -= removed.getDistance();
        }

        blankSegments.add(index, segment2);
        blankSegments.add(index, segment1);
    }


    /**
     * Returns the current cost (distance) of the route in meters.
     *
     * @return Sum of edge distances in the Route.
     */
    double getCost() {
        return cost;
    }

    /**
     * Returns the total score of the route.
     *
     * @return Sum of all attractive arc scores in the Route.
     */
    double getScore() {
        return score;
    }

    /**
     * Converts the Route into a Path object which GraphHopper can display on a map.
     *
     * @return Fully connected Path object
     */
    Path getPath() {
        logger.debug("Route cost: " + getCost());
        Path path = new Path(graph, weighting);

        // If we have a fake arc return no path
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

    /**
     * Returns whether the Route has any arcs in it.
     *
     * @return True if contains arc, else false.
     */
    boolean isEmpty() {
        return getNumArcs() == 0;
    }

    /**
     * Returns a list of the Attractive arcs in the Route, in order.
     *
     * @return Ordered Arc list.
     */
    List<Arc> getArcs() {
        return arcs;
    }

    /**
     * Returns a list of Arcs from the Route whose Improve Potential scores are above the average.
     *
     * @return Arc list.
     */
    List<Arc> getCandidateArcsByIP() {
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

    /**
     * Calculates the Improve Potential score of a given arc.
     *
     * @param arc Arc to calculate
     * @return Improve Potential score.
     */
    private double calcImprovePotential(Arc arc) {
        int v1 = getPrev(arc);
        int v2 = getNext(arc);

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

    /**
     * Returns the Node ID before the specified Arc in the Route.
     *
     * @param a Arc
     * @return Node ID
     */
    int getPrev(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index - 1 >= 0) ? arcs.get(index - 1).adjNode : s;
    }

    /**
     * Returns the Node ID after the specified Arc in the Route.
     *
     * @param a Arc
     * @return Node ID.
     */
    int getNext(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index + 1 <= getNumArcs() - 1) ? arcs.get(index + 1).baseNode : d;
    }

    /**
     * Returns whether the specified Arc is in the Route.
     *
     * @param a Arc to query
     * @return True if arc is in Route, else false.
     */
    boolean contains(@NotNull Arc a) {
        return arcIds.contains(a.edgeId);
    }

    /**
     * Adds the specified arc to the Route at the smallest blank path segment as long as it does not go over the
     * specified budget.
     *
     * @param arc    Arc to insert.
     * @param budget Budget, in meters.
     */
    void insertArcAtMinPathSegment(@NotNull Arc arc, double budget) {
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
