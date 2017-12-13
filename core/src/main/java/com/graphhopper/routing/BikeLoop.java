package com.graphhopper.routing;

import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.BikePriorityWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.sun.istack.internal.NotNull;
import com.sun.istack.internal.Nullable;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static com.graphhopper.util.Parameters.Routing.*;

public class BikeLoop extends AbstractRoutingAlgorithm {

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting bikePriorityWeighting;
    private Graph baseGraph;
    private PMap params;
    private Random random;

    private boolean isFinished = false;
    private double maxCost;
    private double minCost;
    private int maxDepth;
    private int maxIterations;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public BikeLoop(Graph graph, Weighting weighting,
                    EdgeFilter levelEdgeFilter, PMap params) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        this.levelEdgeFilter = levelEdgeFilter;
        bikeEdgeFilter = new DefaultEdgeFilter(flagEncoder);
        bikePriorityWeighting = new BikePriorityWeighting(flagEncoder);
        this.params = params;
        random = new Random();

        parseParams();
    }

    private void parseParams() {
        maxCost = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        minCost = params.getDouble(MIN_DIST, DEFAULT_MIN_DIST);
        maxDepth = params.getInt(SEARCH_DEPTH, DEFAULT_SEARCH_DEPTH);
        maxIterations = params.getInt(MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    private Path runILS(int s, int d) {
        Route solution;
        if (shortestPath(s, d).getDistance() > maxCost) {
            solution = new Route();
        } else {
            solution = initialize(s, d);

            for (int i = 0; i < maxIterations; i++) {

            }
        }

        isFinished = true;

        return getPath(solution, s, d);

    }

    private Route initialize(int s, int d) {
        Route route = new Route();
        // Add fake edge to start solution
        Arc arc = new Arc(-1, s, d, maxCost, 0);
        arc.cas = computeCAS(null, s, d, maxCost);
        route.addArc(0, arc);

        return route;
    }

    private List<Arc> computeCAS(@Nullable List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        // If we don't have a CAS yet, use all edges from the graph as
        // our current CAS
        if (cas == null) {
            cas = getAllArcs();
        }

        for (Arc arc : cas) {
            if (arc.score > 0 && getPathCost(s, d, arc) <= cost) {
                arc.qualityRatio = calcQualityRatio(s, d, arc);
                result.add(arc);
            }
        }

        return result;
    }

    private List<Arc> getAllArcs() {
        List<Arc> arcs = new ArrayList<>();

        EdgeIterator edgeIterator = graph.getAllEdges();
        while (edgeIterator.next()) {

            if (!bikeEdgeFilter.accept(edgeIterator)) {
                continue;
            }

            int edge = edgeIterator.getEdge();
            int baseNode = edgeIterator.getBaseNode();
            int adjNode = edgeIterator.getAdjNode();
            double edgeCost = edgeIterator.getDistance();

            double edgeScore = bikePriorityWeighting
                    .calcWeight(edgeIterator, false, baseNode);

            arcs.add(new Arc(edge, baseNode, adjNode, edgeCost, edgeScore));
        }
        return arcs;
    }

    private List<Arc> updateCAS(@NotNull Arc arc, int v1, int v2, double newBudget,
                                double oldBudget) {
        List<Arc> cas = new ArrayList<>(arc.cas);

        // Restrict CAS using inherit property
        if (newBudget < oldBudget) {
            for (int i = 0; i < cas.size(); i++) {
                Arc e = cas.get(i);
                // Remove any arc whose path is too big
                if (getPathCost(v1, v2, e) > newBudget) {
                    cas.remove(i);
                }
            }
        } else if (newBudget > oldBudget) {
            cas = computeCAS(null, v1, v2, newBudget);
        }

        return cas;
    }

    private double calcQualityRatio(int v1, int v2, @NotNull Arc arc) {
        Path sp1 = shortestPath(v1, arc.baseNode);
        Path sp2 = shortestPath(arc.adjNode, v2);

        double value = 0;

        List<EdgeIteratorState> edges = sp1.calcEdges();
        edges.addAll(sp2.calcEdges());

        for (EdgeIteratorState edge : edges) {
            value += bikePriorityWeighting.calcWeight(edge, false, edge.getBaseNode());
        }

        value += arc.score;

        return value / (sp1.getDistance() + arc.cost + sp2.getDistance());
    }

    private double calcImprovePotential(Arc arc, Route route) {
        int v1 = route.getPrev(arc).adjNode;
        int v2 = route.getNext(arc).baseNode;

        double score = 0;
        double maxDist = 0;

        double dist = getPathCost(v1, v2, arc);

        for (Arc e : arc.cas) {
            score += e.score - arc.score;
            maxDist = Math.max(maxDist, getPathCost(v1, v2, e));
        }

        return score / (maxDist - dist);
    }

    private Path getPath(Route route, int s, int d) {
        return null;
    }

    private Path getPath() {
        return new Path(baseGraph, weighting);
    }

    private static final class Route {
        List<Arc> arcs;
        double cost;
        double score;

        public Route() {
            arcs = new ArrayList<>();
        }

        void addArc(int index, Arc arc) {
            arcs.add(index, arc);
            cost += arc.cost;
            score += arc.score;
        }

        Arc getArc(int index) {
            return arcs.get(index);
        }

        Arc getPrev(Arc arc) {
            int index = arcs.indexOf(arc) - 1;
            if (index >= 0) {
                return arcs.get(index);
            }

            return arc;
        }

        Arc getNext(Arc arc) {
            int index = arcs.indexOf(arc);
            if (index != -1 && index + 1 <= getLength() - 1) {
                return arcs.get(index + 1);
            }

            return arc;
        }

        int getLength() {
            return arcs.size();
        }


    }

    private static class Arc {
        int edgeId, baseNode, adjNode;
        double cost, score, improvePotential, qualityRatio;
        List<Arc> cas;

        Arc(int edgeId, int baseNode, int adjNode, double cost, double score) {
            this.edgeId = edgeId;
            this.baseNode = baseNode;
            this.adjNode = adjNode;
            this.cost = cost;
            this.score = score;
            this.improvePotential = Double.MAX_VALUE;
        }

        @Override
        public String toString() {
            return "Arc{" +
                    "edgeId=" + edgeId +
                    '}';
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            Arc arc = (Arc) o;

            if (edgeId != arc.edgeId) return false;
            if (baseNode != arc.baseNode) return false;
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

    private double getAverageQualityRatio(List<Arc> cas) {
        double avgQR = 0;
        for (Arc ca : cas) {
            avgQR += ca.qualityRatio;
        }
        avgQR /= cas.size();

        return avgQR;
    }

    private Route generatePath(int s, int d, double dist, double minProfit, List<Arc> cas) {
        Arc init = new Arc(-1, s, d, 0, 0);
        Route route = new Route();
        route.addArc(0, init);

        List<Arc> arcs = new ArrayList<>();
        double avgQR = getAverageQualityRatio(cas);
        for (Arc ca : cas) {
            if (ca.qualityRatio >= avgQR) {
                arcs.add(ca);
            }
        }

        while (!arcs.isEmpty() && route.cost < dist) {
            int randomIndex = random.nextInt(arcs.size());
            Arc e = arcs.remove(randomIndex);
            int[] segment = {-1, -1};
            double minDist = Double.MAX_VALUE;

            for (int[] seg : getBlankPathSegments(route)){
                double len = getPathCost(seg[0], seg[1], e);
                if(len < minDist){
                    segment = seg;
                    minDist = len;
                }
            }

            int v1 = segment[0], v2 = segment[1];
            

        }


        if (route.score > minProfit) {
            return route;
        } else {
            return new Route();
        }

    }

    private List<int[]> getBlankPathSegments(Route route) {
        List<int[]> segments = new ArrayList<>();


        for (int i = 0; i < route.getLength() - 2; i++) {
            Arc one = route.getArc(i);
            Arc two = route.getArc(i + 1);

            segments.add(new int[]{one.adjNode, two.adjNode});
        }


        return segments;

    }

    private double getPathCost(int s, int d, @NotNull Arc arc) {
        return shortestPath(s, arc.baseNode).getDistance() + arc.cost +
                shortestPath(arc.adjNode, d).getDistance();
    }

    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    private Path shortestPath(int s, int d) {
        RoutingAlgorithm search =
                new PrepareContractionHierarchies.DijkstraBidirectionCH(graph,
                        weighting, TraversalMode.NODE_BASED)
                        .setEdgeFilter(levelEdgeFilter);

        return search.calcPath(s, d);
    }

    // Unused
    @Override
    public int getVisitedNodes() {
        return 0;
    }

    @Override
    protected boolean finished() {
        return isFinished;
    }

    // Unused
    @Override
    protected Path extractPath() {
        return null;
    }
}
