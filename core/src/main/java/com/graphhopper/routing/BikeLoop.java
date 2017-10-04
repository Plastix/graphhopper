package com.graphhopper.routing;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.ch.PreparationWeighting;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.*;
import com.graphhopper.routing.weighting.BikePriorityWeighting;
import com.graphhopper.routing.weighting.ShortestWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.CHGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;

public class BikeLoop extends AbstractRoutingAlgorithm {

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting shortestWeighting;

    private Graph baseGraph;
    private boolean isFinished = false;
    private int visitedNodes = 0;

    private double maxCost = 50_000; // in meters
    private double minCost = 16_000; // in meters

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public BikeLoop(Graph graph) {
        super(graph, new BikePriorityWeighting(new RacingBikeFlagEncoder()), TraversalMode.EDGE_BASED_1DIR);

        if (!(graph instanceof CHGraph)) {
            throw new IllegalArgumentException("You must pass a CH graph to this algorithm!");
        }
        baseGraph = graph.getBaseGraph();
        levelEdgeFilter = new LevelEdgeFilter((CHGraph) graph);
        bikeEdgeFilter = new DefaultEdgeFilter(weighting.getFlagEncoder());
        shortestWeighting = new PreparationWeighting(new ShortestWeighting(weighting.getFlagEncoder()));
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    // TODO (Aidan)
    private Path runILS(int s, int d) {
//        Path path = new Path(graph, weighting);
//        path.setFromNode(s)
//                .setEndNode(d);

        isFinished = true;

        return null;
    }

    // TODO (Aidan)
    private boolean localSearch(Solution path, int s, int d, double dist,
                                double minProfit) {
        if (shortestPath(s, d) > dist) {
            return false;
        }

        // Make sure to use baseGraph for traversal (non-CH version)
        EdgeExplorer explorer = baseGraph.createEdgeExplorer(bikeEdgeFilter);
        EdgeIterator iter = explorer.setBaseNode(s);

        while (iter.next()) {
            int currentEdge = iter.getEdge();
            double currentDist = iter.getDistance();
            int nextNode = iter.getAdjNode();

            // Priority of the road
            double edgeScore = weighting.calcWeight(iter, false, nextNode);

            path.addEdge(currentEdge, currentDist, edgeScore);

            if (nextNode == d && path.cost >= minCost && path.score > minProfit) {
                return true;
            } else {
                if (localSearch(path, nextNode, d, dist - currentDist, minProfit)) {
                    return true;
                }
            }

            path.removeEdge(currentEdge, currentDist, edgeScore);
        }

        return false;
    }

    private static final class Solution {
        IntArrayList path;
        double cost;
        double score;

        public Solution() {
            path = new IntArrayList();
            cost = 0;
        }

        void addEdge(int edgeId, double cost, double score) {
            path.add(edgeId);
            this.cost += cost;
            this.score += score;
        }

        void removeEdge(int edgeId, double cost, double score) {
            path.removeFirst(edgeId);
            this.cost -= cost;
            this.score -= score;
        }

    }


    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    private double shortestPath(int s, int d) {
        PrepareContractionHierarchies.DijkstraBidirectionCH search =
                new PrepareContractionHierarchies.DijkstraBidirectionCH(graph,
                        shortestWeighting, TraversalMode.EDGE_BASED_2DIR);
        // TODO (Aidan) Replace this edge filter with one that respects vehicles
        search.setEdgeFilter(levelEdgeFilter);

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
