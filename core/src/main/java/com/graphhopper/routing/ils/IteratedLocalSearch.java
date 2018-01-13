package com.graphhopper.routing.ils;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.GHPoint3D;
import com.graphhopper.util.shapes.Shape;
import com.sun.istack.internal.NotNull;
import com.sun.istack.internal.Nullable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static com.graphhopper.util.Parameters.Routing.*;

/**
 * Routing Algorithm which implements the bike route Iterated Local Search algorithm from the following paper:
 * https://dl.acm.org/citation.cfm?id=2820835
 */
public class IteratedLocalSearch extends AbstractRoutingAlgorithm implements ShortestPathCalculator {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    // Constants passed in as parameters
    private final double MIN_ROAD_SCORE;
    private final int MIN_ROAD_LENGTH;
    private final double MAX_COST;
    private final int MAX_ITERATIONS;
    private final long SEED;

    private Graph baseGraph;
    private LocationIndex locationIndex;
    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter edgeFilter; // Determines which edges are considered in CAS
    private Weighting bikePriorityWeighting; // Used for scoring arcs
    private Random random;
    private int s, d; // Start and End Node IDs


    private boolean isFinished = false;

    /**
     * Creates a new ILS algorithm instance.
     *
     * @param graph           Graph to run algorithm on.
     * @param weighting       Weighting to calculate costs.
     * @param levelEdgeFilter Edge filter for CH shortest path computation
     * @param params          Parameters map.
     * @param locationIndex   Location Index of graph.
     */
    public IteratedLocalSearch(Graph graph, Weighting weighting,
                               EdgeFilter levelEdgeFilter, PMap params, LocationIndex locationIndex) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        this.levelEdgeFilter = levelEdgeFilter;
        this.locationIndex = locationIndex;
        edgeFilter = new DefaultEdgeFilter(flagEncoder);
        bikePriorityWeighting = new BikePriorityWeighting(flagEncoder);

        MAX_COST = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        MAX_ITERATIONS = params.getInt(Parameters.Routing.MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);
        MIN_ROAD_SCORE = params.getDouble(Parameters.Routing.MIN_ROAD_SCORE, DEFAULT_MIN_ROAD_SCORE);
        MIN_ROAD_LENGTH = params.getInt(Parameters.Routing.MIN_ROAD_LENGTH, DEFAULT_MIN_ROAD_LENGTH);
        SEED = params.getLong(Parameters.Routing.SEED, System.currentTimeMillis());

        random = new Random(SEED);
    }

    /**
     * Calculates a route between the specified node IDs.
     *
     * @param from Start Node ID.
     * @param to   End Node ID.
     * @return Path
     */
    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        s = from;
        d = to;
        return runILS();
    }

    /**
     * Main algorithm loop
     */
    private Path runILS() {
        Route solution;
        if(shortestPath(s, d).getDistance() > MAX_COST) {
            solution = Route.newRoute(this, baseGraph, weighting, s, d, MAX_COST);
        } else {
            solution = initializeSolution();

            logger.info("Seed: " + SEED);
            for(int i = 0; i < MAX_ITERATIONS; i++) {
                logger.debug("Iteration " + i);
                List<Arc> arcs = solution.getCandidateArcsByIP();
                logger.debug("Possible arcs to remove from solution: " + arcs.size());

                int randomIndex = random.nextInt(arcs.size());
                Arc e = arcs.remove(randomIndex);

                double b1 = solution.getRemainingCost() + e.cost; // Remaining budget after removing e from solution

                Route path = generatePath(solution.getPrev(e), solution.getNext(e), b1, e.score, e.getCas());

                if(!path.isEmpty()) {
                    logger.debug("Found path with with dist " + path.getCost());
                    int index = solution.removeArc(e);
                    solution.insertRoute(index, path);
                    for(Arc arc : solution) {
                        double b2 = solution.getRemainingCost() + arc.cost; // Remaining budget after removing arc from solution

                        int startCAS = solution.getPrev(arc);
                        int endCAS = solution.getNext(arc);

                        if(path.contains(arc) || arc.adjNode == startCAS || arc.baseNode == endCAS) {
                            // Using removed arc's CAS to compute next CAS (inherit)
                            arc.setCas(computeCAS(e.getCas(), startCAS, endCAS, b2));
                        } else {
                            arc.setCas(updateCAS(arc, startCAS, endCAS, b1, b2));
                        }
                    }
                }
            }
        }

        isFinished = true;

        return solution.getPath();
    }

    /**
     * Creates a new Route, adds a fake arc, and computes first CAS.
     *
     * @return Route.
     */
    private Route initializeSolution() {
        Route route = Route.newRoute(this, baseGraph, weighting, s, d, MAX_COST);
        // Add fake edge to start solution
        Arc arc = new Arc(Arc.FAKE_ARC_ID, s, d, MAX_COST, 0, PointList.EMPTY);
        arc.setCas(computeCAS(null, s, d, MAX_COST));
        route.addArc(0, arc);

        return route;
    }

    /**
     * Computes the Candidate Arc Set for the specified start, end, and cost parameters.
     *
     * @param cas  Current CAS. May be null.
     * @param s    Start Node ID.
     * @param d    End Node Id.
     * @param cost Cost allowance.
     * @return CAS
     */
    private List<Arc> computeCAS(@Nullable List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        GHPoint focus1 = new GHPoint(nodeAccess.getLatitude(s), nodeAccess.getLongitude(s));
        GHPoint focus2 = new GHPoint(nodeAccess.getLatitude(d), nodeAccess.getLongitude(d));
        Shape ellipse = new Ellipse(focus1, focus2, cost);

        // If we don't have a CAS yet
        // Fetch arcs from the graph using spatial indices
        if(cas == null) {
            cas = getAllArcs(ellipse);
        }

        logger.debug("Starting to compute CAS! num arcs: " + cas.size() + " cost: " + cost);

        outer:
        for(Arc arc : cas) {

            // Basic restrictions on attractive arcs
            if(arc.score < MIN_ROAD_SCORE || arc.cost < MIN_ROAD_LENGTH) {
                continue;
            }

            // Spatial-based feasibility checking
            for(GHPoint3D ghPoint3D : arc.points) {
                if(!ellipse.contains(ghPoint3D.lat, ghPoint3D.lon)) {
                    continue outer;
                }
            }

            // Check arc feasibility
            if(getPathCost(s, d, arc) <= cost) {
                calcQualityRatio(arc, s, d);
                result.add(arc);
            }
        }

        logger.debug("Finished computing CAS! size: " + result.size());

        return result;
    }

    /**
     * Fetches all Arcs from the graph which are contained inside of the specified Shape.
     *
     * @param shape Shape.
     * @return Arc list.
     */
    private List<Arc> getAllArcs(final Shape shape) {
        logger.debug("Fetching arcs from graph!");
        final List<Arc> arcs = new ArrayList<>();

        GHPoint center = shape.getCenter();
        QueryResult qr = locationIndex.findClosest(center.getLat(), center.getLon(), edgeFilter);
        // TODO: if there is no street close to the center it'll fail although there are roads covered. Maybe we should check edge points or some random points in the Shape instead?
        if(!qr.isValid())
            throw new IllegalArgumentException("Shape " + shape + " does not cover graph");

        if(shape.contains(qr.getSnappedPoint().lat, qr.getSnappedPoint().lon)) {
            arcs.add(getArc(qr.getClosestEdge()));
        }

        BreadthFirstSearch bfs = new BreadthFirstSearch() {
            final Shape localShape = shape;
            final IntHashSet edgeIds = new IntHashSet();

            @Override
            protected boolean goFurther(int nodeId) {
                return localShape.contains(nodeAccess.getLatitude(nodeId), nodeAccess.getLongitude(nodeId));
            }

            @Override
            protected boolean checkAdjacent(EdgeIteratorState edge) {
                if(localShape.contains(nodeAccess.getLatitude(edge.getAdjNode()), nodeAccess.getLongitude(edge.getAdjNode()))) {
                    int edgeId = edge.getEdge();
                    if(!edgeIds.contains(edgeId)) {
                        arcs.add(getArc(edge));
                        edgeIds.add(edgeId);
                    }
                    return true;
                }
                return false;
            }
        };


        bfs.start(baseGraph.createEdgeExplorer(edgeFilter), qr.getClosestNode());

        logger.debug("Got all arcs inside of ellipse! num: " + arcs.size());

        return arcs;
    }

    /**
     * Returns an Arc object instance from the specified EdgeIterator from the Graph.
     *
     * @param edgeIterator Edge
     * @return Arc
     */
    private Arc getArc(EdgeIteratorState edgeIterator) {
        int edge = edgeIterator.getEdge();
        int baseNode = edgeIterator.getBaseNode();
        int adjNode = edgeIterator.getAdjNode();
        double edgeCost = edgeIterator.getDistance();

        double edgeScore = bikePriorityWeighting
                .calcWeight(edgeIterator, false, baseNode);

        return new Arc(edge, baseNode, adjNode, edgeCost, edgeScore, edgeIterator.fetchWayGeometry(0));
    }

    /**
     * Updates the Candidate Arc Set for the specified Arc.
     *
     * @param arc       Arc to update.
     * @param s         Start Node Id.
     * @param d         End Node Id.
     * @param newBudget New allowable budget.
     * @param oldBudget Old allowable budget.
     * @return New CAS
     */
    private List<Arc> updateCAS(@NotNull Arc arc, int s, int d, double newBudget,
                                double oldBudget) {
        List<Arc> cas = new ArrayList<>(arc.getCas());

        // Restrict CAS using inherit property
        if(newBudget < oldBudget) {
            for(int i = 0; i < cas.size(); i++) {
                Arc e = cas.get(i);
                // Remove any arc whose path is too big
                if(getPathCost(s, d, e) > newBudget) {
                    cas.remove(i);
                }
            }
        } else if(newBudget > oldBudget) {
            cas = computeCAS(null, s, d, newBudget);
        }

        return cas;
    }

    /**
     * Computes the Quality Ratio for the specified Arc.
     *
     * @param arc Arc.
     * @param s   Start Node ID.
     * @param d   End Node ID.
     */
    private void calcQualityRatio(@NotNull Arc arc, int s, int d) {
        Path sp1 = shortestPath(s, arc.baseNode);
        Path sp2 = shortestPath(arc.adjNode, d);

        double value = 0;

        List<EdgeIteratorState> edges = sp1.calcEdges();
        edges.addAll(sp2.calcEdges());

        for(EdgeIteratorState edge : edges) {
            value += bikePriorityWeighting.calcWeight(edge, false, edge.getBaseNode());
        }

        value += arc.score;

        arc.qualityRatio = value / (sp1.getDistance() + arc.cost + sp2.getDistance());
    }

    /**
     * Returns a list of Arcs from the specified CAS whose Quality Ratio scores are above the average.
     *
     * @param cas CAS
     * @return Arc list.
     */
    private List<Arc> getCandidateArcsByQR(List<Arc> cas) {
        List<Arc> arcs = new ArrayList<>();
        double avgQR = 0;
        for(Arc ca : cas) {
            avgQR += ca.qualityRatio;
        }
        avgQR /= cas.size();

        for(Arc ca : cas) {
            if(ca.qualityRatio >= avgQR) {
                arcs.add(ca);
            }
        }

        return arcs;
    }

    /**
     * Generates a Route from the specified start and end nodes whose distance is less than the specified budget and
     * total score is above the specified.
     *
     * @param s         Start Node Id.
     * @param d         End Node Id.
     * @param dist      Allowable budget.
     * @param minProfit Minimum required score.
     * @param cas       CAS
     * @return Route. May be empty!
     */
    private Route generatePath(int s, int d, double dist, double minProfit, List<Arc> cas) {
        logger.debug("Generating path! dist: " + dist + " minProfit: " + minProfit + " cas size: " + cas.size());
        Route route = Route.newRoute(this, baseGraph, weighting, s, d, dist);

        List<Arc> arcs = getCandidateArcsByQR(cas);
        while(!arcs.isEmpty() && route.getCost() < dist) {
            int randomIndex = random.nextInt(arcs.size());
            Arc e = arcs.remove(randomIndex);
            route.insertArcAtMinPathSegment(e, dist);
        }

        if(route.getScore() > minProfit) {
            return route;
        } else {
            return Route.newRoute(this, baseGraph, weighting, s, d, dist);
        }

    }

    @Override
    public double getPathCost(int s, int d, @NotNull Arc arc) {
        return shortestPath(s, arc.baseNode).getDistance() + arc.cost +
                shortestPath(arc.adjNode, d).getDistance();
    }

    @Override
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
