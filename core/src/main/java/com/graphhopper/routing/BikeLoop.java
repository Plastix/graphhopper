package com.graphhopper.routing;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.coll.GHBitSet;
import com.graphhopper.coll.GHBitSetImpl;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.BikePriorityWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;

public class BikeLoop extends AbstractRoutingAlgorithm {

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting bikePriorityWeighting;

    private Graph baseGraph;
    private boolean isFinished = false;
    private int visitedNodes = 0;

    private double maxCost = 50_000; // in meters
    private double minCost = 8_000; // in meters
    private int maxDepth = 20;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public BikeLoop(Graph graph, Weighting weighting, EdgeFilter levelEdgeFilter) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        this.levelEdgeFilter = levelEdgeFilter;
        bikeEdgeFilter = new DefaultEdgeFilter(flagEncoder);
        bikePriorityWeighting = new BikePriorityWeighting(flagEncoder);
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    private Path runILS(int s, int d) {
        Route route = new Route();
        boolean found = localSearch(route, s, d, maxCost, 0, maxDepth);

        isFinished = true;

        if (found) {
            return getPath(route, s, d);
        } else {
            return getPath();
        }
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

            if (route.bitSet.contains(currentEdge)) {
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

            route.addEdge(currentEdge, nextNode, edgeCost, edgeScore);

            if (nextNode == d &&
                    route.cost >= minCost &&
                    route.score > minProfit) {
                return true;
            } else if (localSearch(route, nextNode, d, remainingDist,
                    minProfit, maxDepth - 1)) {
                return true;
            }

            route.removeEdge(currentEdge, nextNode, edgeCost, edgeScore);
        }

        return false;
    }

    private Path getPath(Route route, int s, int d) {
        Path path = getPath();
        for (int i = 0; i < route.edges.size(); i++) {
            path.addEdge(route.edges.get(i));
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
        IntArrayList edges;
        IntArrayList nodes;
        GHBitSet bitSet;
        double cost;
        double score;

        Route() {
            edges = new IntArrayList();
            nodes = new IntArrayList();
            bitSet = new GHBitSetImpl();
        }

        void addEdge(int edgeId, int nodeId, double cost, double score) {
            edges.add(edgeId);
            nodes.add(nodeId);
            bitSet.add(edgeId);
            this.cost += cost;
            this.score += score;
        }

        void removeEdge(int edgeId, int nodeId, double cost, double score) {
            edges.removeFirst(edgeId);
            nodes.removeFirst(nodeId);
            bitSet.remove(edgeId);
            this.cost -= cost;
            this.score -= score;
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

    @Override
    public int getVisitedNodes() {
        return visitedNodes;
    }

    @Override
    protected boolean finished() {
        return isFinished;
    }

    // Unused by this algorithm
    @Override
    protected Path extractPath() {
        return null;
    }
}
