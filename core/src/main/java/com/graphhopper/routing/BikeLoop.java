package com.graphhopper.routing;

import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.BikePriorityWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
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
    // TODO (Aidan) remove magic number
    private int maxNoImprove = 5;

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
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    private Path runILS(int s, int d) {
        Route solution = initialize(s, d);

//        solution = improve(solution, s, d, maxNoImprove);

        isFinished = true;

        return getPath(solution, s, d);

    }

    private Route improve(Route solution, int s, int d, int maxNoImprove) {
        Route temp = solution.copy();
        Route newPath = new Route();
        int a = 1, r = 1, noImprove = 0;
        while (noImprove < maxNoImprove) {
            int size = temp.edges.size();

            if (r > size) {
                r = 1;
            }

            if (a + r > size - 1) {
                r = size - 1 - a;
            }

            // Remove arcs a - r
            double minScore = 0;
            int startId = s, endId = d;
            for (int i = 0; i < r; i++) {
                Route.Arc arc = temp.removeEdgeIndex(a + i - 1);
                minScore += arc.score;

                if (i == 0) {
                    startId = arc.baseNode;
                }

                if (i == r - 1) {
                    endId = arc.adjNode;
                }
            }

            if (localSearch(newPath, startId, endId, maxCost - solution.cost,
                    minScore, maxDepth)) {
                temp.splice(newPath, a - 1);
                noImprove = 0;
                a = 1;
                r = 1;
            } else {
                newPath.clear();
                noImprove++;
                a++;
                r++;
            }
        }

        return temp;
    }

    private Route initialize(int s, int d) {
        Route route = new Route();

        if (!localSearch(route, s, d, maxCost, 0, maxDepth)) {
            route.clear();
        }

        return route;
    }

    private boolean localSearch(Route route, int s, int d, double dist,
                                double minProfit, int maxDepth) {
        if (maxDepth == 0) {
            return false;
        }

        // Make sure to use baseGraph for traversal (non-CH version)
        EdgeExplorer explorer = baseGraph.createEdgeExplorer(bikeEdgeFilter);
        EdgeIterator iter = explorer.setBaseNode(s);

        while (iter.next()) {
            int currentEdge = iter.getEdge();

            if (route.bitSet.get(currentEdge)) {
                continue;
            }

            double edgeCost = iter.getDistance();
            int nextNode = iter.getAdjNode();

            double remainingDist = dist - edgeCost;
            double shortestDist = shortestPath(nextNode, d);

            if (shortestDist >= remainingDist) {
                continue;
            }

            double edgeScore = bikePriorityWeighting
                    .calcWeight(iter, false, nextNode);

            route.addEdge(currentEdge, s, nextNode, edgeCost, edgeScore);

            if (nextNode == d &&
                    route.cost >= minCost &&
                    route.score > minProfit) {
                return true;
            } else if (localSearch(route, nextNode, d, remainingDist,
                    minProfit, maxDepth - 1)) {
                return true;
            }

            route.removeEdge(currentEdge);
        }

        return false;
    }

    private Path getPath(Route route, int s, int d) {
        int numEdges = route.edges.size();
        Path path = getPath();
        for (int i = 0; i < numEdges; i++) {
            Route.Arc arc = route.edges.get(i);
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


        void addEdge(int edgeId, int baseNode, int adjNode, double cost, double score) {
            edges.add(new Arc(edgeId, baseNode, adjNode, cost, score));
            bitSet.set(edgeId);
            this.cost += cost;
            this.score += score;
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
            score = arc.score;
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


        private class Arc {
            int edgeId, baseNode, adjNode;
            double cost, score;

            Arc(int edgeId, int baseNode, int adjNode, double cost, double score) {
                this.edgeId = edgeId;
                this.baseNode = baseNode;
                this.adjNode = adjNode;
                this.cost = cost;
                this.score = score;
            }
        }
    }


    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    private double shortestPath(int s, int d) {
        RoutingAlgorithm search =
                new PrepareContractionHierarchies.DijkstraBidirectionCH(graph,
                        weighting, TraversalMode.NODE_BASED)
                        .setEdgeFilter(levelEdgeFilter);

        Path path = search.calcPath(s, d);
        return path.getDistance();
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
