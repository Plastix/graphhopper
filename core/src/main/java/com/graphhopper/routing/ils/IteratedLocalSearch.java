package com.graphhopper.routing.ils;

import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;

import static com.graphhopper.util.Parameters.Routing.*;

/**
 * Routing Algorithm which implements the bike route Iterated Local Search algorithm from the following paper:
 * https://www.sciencedirect.com/science/article/pii/S1366554514000751
 */
public class IteratedLocalSearch extends AbstractRoutingAlgorithm {

    private final double MAX_COST;
    private final double MIN_COST;
    private final int MAX_DEPTH;
    private final int MAX_ITERATIONS;

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting bikePriorityWeighting;
    private Graph baseGraph;

    private boolean isFinished = false;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public IteratedLocalSearch(Graph graph, Weighting weighting,
                               EdgeFilter levelEdgeFilter, PMap params) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        this.levelEdgeFilter = levelEdgeFilter;
        bikeEdgeFilter = new DefaultEdgeFilter(flagEncoder);
        bikePriorityWeighting = new BikePriorityWeighting(flagEncoder);

        MAX_COST = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        MIN_COST = params.getDouble(MIN_DIST, DEFAULT_MIN_DIST);
        MAX_DEPTH = params.getInt(SEARCH_DEPTH, DEFAULT_SEARCH_DEPTH);
        MAX_ITERATIONS = params.getInt(Parameters.Routing.MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    private Path runILS(int s, int d) {
        Route solution = initialize(s, d);
        solution = improve(solution, s, d);
        isFinished = true;
        return getPath(solution, s, d);
    }

    private Route improve(Route solution, int s, int d) {
        Route newPath = new Route();
        int a = 1, r = 1, count = 0;
        while(count < MAX_ITERATIONS) {
            Route temp = solution.copy();
            int size = temp.edges.size();

            if(r > size) {
                r = 1;
            }

            if(a + r > size - 1) {
                r = size - 1 - a;
            }

            // Remove arcs a - r
            double minScore = 0;
            int startId = s, endId = d;
            for(int i = 0; i < r; i++) {
                Arc arc = temp.removeEdgeIndex(a - 1);
                minScore += arc.score;

                if(i == 0) {
                    startId = arc.baseNode;
                }

                if(i == r - 1) {
                    endId = arc.adjNode;
                }
            }

            // Don't allow search to traverse roads already in our path
            newPath.blacklist(temp);
            if(localSearch(newPath, startId, endId, MAX_COST - temp.cost,
                    minScore, MAX_DEPTH)) {
                temp.splice(newPath, a - 1);
                solution = temp;
                a = 1;
                r = 1;
            } else {
                a++;
                r++;
            }
            // Clear temp path so we can use it again
            newPath.clear();
            count++;
        }

        return solution;
    }

    private Route initialize(int s, int d) {
        Route route = new Route();

        if(!localSearch(route, s, d, MAX_COST, 0, MAX_DEPTH)) {
            route.clear();
        }

        return route;
    }

    private boolean localSearch(Route route, int s, int d, double dist,
                                double minProfit, int maxDepth) {
        if(maxDepth == 0) {
            return false;
        }

        // Make sure to use baseGraph for traversal (non-CH version)
        EdgeExplorer explorer = baseGraph.createEdgeExplorer(bikeEdgeFilter);
        EdgeIterator iter = explorer.setBaseNode(s);

        while(iter.next()) {
            int currentEdge = iter.getEdge();

            if(route.bitSet.get(currentEdge)) {
                continue;
            }

            double edgeCost = iter.getDistance();
            int nextNode = iter.getAdjNode();

            double remainingDist = dist - edgeCost;
            double shortestDist = shortestPath(nextNode, d);

            if(shortestDist >= remainingDist) {
                continue;
            }

            double edgeScore = bikePriorityWeighting
                    .calcWeight(iter, false, nextNode);

            route.addEdge(currentEdge, s, nextNode, edgeCost, edgeScore);

            if(nextNode == d &&
                    route.cost >= MIN_COST &&
                    route.score > minProfit) {
                return true;
            } else if(localSearch(route, nextNode, d, remainingDist,
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
        for(int i = 0; i < numEdges; i++) {
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
