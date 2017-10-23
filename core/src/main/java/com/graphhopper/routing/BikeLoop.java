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

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

import static com.graphhopper.util.Parameters.Routing.*;

public class BikeLoop extends AbstractRoutingAlgorithm {

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting bikePriorityWeighting;
    private Graph baseGraph;
    private PMap params;

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
            // TODO (Aidan)
            solution = initialize(s, d);
        }

        isFinished = true;

        return getPath(solution, s, d);

    }

    private Route initialize(int s, int d) {
        Route route = new Route();
        // Add fake edge to start solution
        Arc arc = new Arc(-1, s, d, maxCost, 0,
                computeCAS(null, s, d, maxCost));

        route.addArc(arc);

        return route;
    }

    private List<Arc> computeCAS(List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        // If we don't have a CAS yet, use all edges from the graph
        if (cas == null) {
            cas = new ArrayList<>();
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

                cas.add(new Arc(edge, baseNode, adjNode, edgeCost, edgeScore, null));

            }
        }

        for (Arc arc : cas) {
            if (arc.score > 0 && fullPathCost(s, d, arc) <= cost) {
                arc.qualityRatio = getQualityRatio(s, d, arc);
                result.add(arc);
            }
        }
        return result;
    }

    private double getQualityRatio(int s, int d, Arc arc) {
        Path sp1 = shortestPath(s, arc.baseNode);
        Path sp2 = shortestPath(arc.adjNode, d);

        double value = 0;

        List<EdgeIteratorState> edges = sp1.calcEdges();
        edges.addAll(sp2.calcEdges());

        for (EdgeIteratorState edge : edges) {
            value += bikePriorityWeighting.calcWeight(edge, false, edge.getBaseNode());
        }

        return value / (sp1.getDistance() + arc.cost + sp2.getDistance());
    }

    private double getImprovePotential(Arc arc) {
        if (arc.cas == null) {
            return 0;
        }

        double sum = 0;
        double max = Double.MIN_VALUE;

        for (Arc casArc : arc.cas) {
            sum += (casArc.score - arc.score);


        }

        return sum / (max -;
    }

    private Path getPath(Route route, int s, int d) {
        int numEdges = route.edges.size();
        Path path = getPath();
        for (int i = 0; i < numEdges; i++) {
            Arc arc = route.edges.get(i);
            path.processEdge(arc.edgeId, arc.adjNode, arc.edgeId);
        }
        return path
                .setEndNode(d)
                .setFromNode(s)
                .setFound(!route.edges.isEmpty());
    }

    private Path getPath() {
        return new Path(baseGraph, weighting);
    }

    private static final class Route {
        List<Arc> edges;
        BitSet bitSet;
        double cost;
        double score;

        Route() {
            edges = new ArrayList<>();
            bitSet = new BitSet();
        }

        // Copy constructor
        Route(Route route) {
            cost = route.cost;
            score = route.score;
            edges = new ArrayList<>(route.edges);
            bitSet = (BitSet) route.bitSet.clone();
        }

        void addArc(Arc arc) {
            edges.add(arc);
            bitSet.set(arc.edgeId);
            this.cost += arc.cost;
            this.score += arc.score;
        }

        void removeEdge(int edgeId) {
            for (int i = edges.size() - 1; i >= 0; i--) {
                Arc arc = edges.get(i);
                if (arc.edgeId == edgeId) {
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
            for (Arc edge : edges) {
                stringBuilder.append("(")
                        .append(edge.baseNode)
                        .append(",")
                        .append(edge.adjNode)
                        .append(") ->");
            }
            return stringBuilder.toString();
        }


    }

    private static class Arc {
        int edgeId, baseNode, adjNode;
        double cost, score, improvePotential, qualityRatio;
        List<Arc> cas;

        Arc(int edgeId, int baseNode, int adjNode, double cost, double score, List<Arc> cas) {
            this.edgeId = edgeId;
            this.baseNode = baseNode;
            this.adjNode = adjNode;
            this.cost = cost;
            this.score = score;
            this.cas = cas;
            this.improvePotential = Double.MAX_VALUE;
        }

        @Override
        public String toString() {
            return "Arc{" +
                    "edgeId=" + edgeId +
                    '}';
        }
    }

    private double fullPathCost(int s, int d, Arc arc) {
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
