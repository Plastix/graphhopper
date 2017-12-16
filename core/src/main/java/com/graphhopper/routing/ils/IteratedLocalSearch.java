package com.graphhopper.routing.ils;

import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
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

public class IteratedLocalSearch extends AbstractRoutingAlgorithm implements ShortestPathCalculator {

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
    public IteratedLocalSearch(Graph graph, Weighting weighting,
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
        if(shortestPath(s, d).getDistance() > maxCost) {
            solution = Route.newRoute(this);
        } else {
            solution = initialize(s, d);

            for(int i = 0; i < maxIterations; i++) {
                for(Arc arc : solution.getArcs()) {
                    arc.improvePotential = calcImprovePotential(arc, solution);
                }

                List<Arc> arcs = new ArrayList<>();
                double avgIP = getAverageImprovePotential(solution.getArcs());
                for(Arc ca : solution.getArcs()) {
                    if(ca.qualityRatio >= avgIP) {
                        arcs.add(ca);
                    }
                }

                int index = random.nextInt(arcs.size());
                Arc e = arcs.remove(index);

                double b1 = solution.getCost() + e.cost; // Budget after removing e from solution

                Route path = generatePath(solution.getPrev(e).adjNode, solution.getNext(e).baseNode, b1, e.score, e.getCas());

                if(path.getNumArcs() != 0) {
                    solution.removeArc(e);
                    // TODO index??
//                    solution.insertRoute();
                    for(Arc arc : solution.getArcs()) {
                        double b2 = solution.getCost() + arc.cost; // Budget after removing a from solution

                        Arc prev = solution.getPrev(arc);
                        Arc next = solution.getNext(arc);
                        if(solution.contains(arc) || arc.equals(prev) || arc.equals(next)){
                            arc.setCas(computeCAS(null, prev.adjNode, next.baseNode, b2));
                        }else{
                            // TODO budgets???
                            arc.setCas(updateCAS(arc, prev.adjNode, next.baseNode, b1, b2));
                        }
                    }
                }
            }
        }

        isFinished = true;

        return solution.getPath();
    }

    private Route initialize(int s, int d) {
        Route route = Route.newRoute(this);
        // Add fake edge to start solution
        Arc arc = new Arc(-1, s, d, maxCost, 0);
        arc.setCas(computeCAS(null, s, d, maxCost));
        route.addArc(0, arc);

        return route;
    }

    private List<Arc> computeCAS(@Nullable List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        // If we don't have a CAS yet, use all edges from the graph as
        // our current CAS
        if(cas == null) {
            cas = getAllArcs();
        }

        for(Arc arc : cas) {
            if(arc.score > 0 && getPathCost(s, d, arc) <= cost) {
                arc.qualityRatio = calcQualityRatio(s, d, arc);
                result.add(arc);
            }
        }

        return result;
    }

    private List<Arc> getAllArcs() {
        List<Arc> arcs = new ArrayList<>();

        EdgeIterator edgeIterator = graph.getAllEdges();
        while(edgeIterator.next()) {

            if(!bikeEdgeFilter.accept(edgeIterator)) {
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
        List<Arc> cas = new ArrayList<>(arc.getCas());

        // Restrict CAS using inherit property
        if(newBudget < oldBudget) {
            for(int i = 0; i < cas.size(); i++) {
                Arc e = cas.get(i);
                // Remove any arc whose path is too big
                if(getPathCost(v1, v2, e) > newBudget) {
                    cas.remove(i);
                }
            }
        } else if(newBudget > oldBudget) {
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

        for(EdgeIteratorState edge : edges) {
            value += bikePriorityWeighting.calcWeight(edge, false, edge.getBaseNode());
        }

        value += arc.score;

        return value / (sp1.getDistance() + arc.cost + sp2.getDistance());
    }

    private double calcImprovePotential(Arc arc, Route route) {
        // TODO (Aidan)
        int v1 = route.getPrev(arc).adjNode;
        int v2 = route.getNext(arc).baseNode;

        double score = 0;
        double maxDist = 0;

        double dist = getPathCost(v1, v2, arc);

        for(Arc e : arc.getCas()) {
            score += e.score - arc.score;
            maxDist = Math.max(maxDist, getPathCost(v1, v2, e));
        }

        return score / (maxDist - dist);
    }

    private double getAverageQualityRatio(List<Arc> cas) {
        double avgQR = 0;
        for(Arc ca : cas) {
            avgQR += ca.qualityRatio;
        }
        avgQR /= cas.size();

        return avgQR;
    }

    private double getAverageImprovePotential(List<Arc> cas) {
        double avgIP = 0;
        for(Arc ca : cas) {
            avgIP += ca.improvePotential;
        }
        avgIP /= cas.size();

        return avgIP;
    }

    private Route generatePath(int s, int d, double dist, double minProfit, List<Arc> cas) {
        Arc init = new Arc(-1, s, d, 0, 0);
        Route route = Route.newRoute(this);
        route.addArc(0, init);

        List<Arc> arcs = new ArrayList<>();
        double avgQR = getAverageQualityRatio(cas);
        for(Arc ca : cas) {
            if(ca.qualityRatio >= avgQR) {
                arcs.add(ca);
            }
        }

        while(!arcs.isEmpty() && route.getCost() < dist) {
            int randomIndex = random.nextInt(arcs.size());
            Arc e = arcs.remove(randomIndex);

            Route.Segment minSegment = route.getSmallestSegment();

            if(getPathCost(minSegment.startNode, minSegment.endNode, e) <=
                    dist - route.getCost() + minSegment.cost) {
                route.addArc(minSegment.arcIndex, e);
            }
        }


        if(route.getScore() > minProfit) {
            return route;
        } else {
            return Route.newRoute(this);
        }

    }

    private double getPathCost(int s, int d, @NotNull Arc arc) {
        return shortestPath(s, arc.baseNode).getDistance() + arc.cost +
                shortestPath(arc.adjNode, d).getDistance();
    }

    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    public Path shortestPath(int s, int d) {
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
