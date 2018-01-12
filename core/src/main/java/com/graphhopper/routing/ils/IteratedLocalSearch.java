package com.graphhopper.routing.ils;

import com.graphhopper.coll.GHBitSet;
import com.graphhopper.coll.GHBitSetImpl;
import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.BikePriorityWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.RAMDirectory;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.BreadthFirstSearch;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;
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

public class IteratedLocalSearch extends AbstractRoutingAlgorithm implements ShortestPathCalculator {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    // Constants passed in as parameters
    private final double MIN_ROAD_SCORE;
    private final int MIN_ROAD_LENGTH;
    private final double MAX_COST;
    private final int MAX_ITERATIONS;

    private Graph baseGraph;
    private LocationIndex locationIndex;
    private NodeAccess nodeAccess;
    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private EdgeFilter bikeEdgeFilter;
    private Weighting bikePriorityWeighting;
    private Random random;

    private boolean isFinished = false;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public IteratedLocalSearch(Graph graph, Weighting weighting,
                               EdgeFilter levelEdgeFilter, PMap params) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        nodeAccess = baseGraph.getNodeAccess();
        this.levelEdgeFilter = levelEdgeFilter;
        bikeEdgeFilter = new DefaultEdgeFilter(flagEncoder);
        bikePriorityWeighting = new BikePriorityWeighting(flagEncoder);
        random = new Random();

        // TODO (Aidan) Uber hackyness
        locationIndex = new LocationIndexTree(((QueryGraph) baseGraph).getMainGraph().getBaseGraph(), new RAMDirectory())
                .prepareIndex();

        MAX_COST = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        MAX_ITERATIONS = params.getInt(Parameters.Routing.MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);
        MIN_ROAD_SCORE = params.getDouble(Parameters.Routing.MIN_ROAD_SCORE, DEFAULT_MIN_ROAD_SCORE);
        MIN_ROAD_LENGTH = params.getInt(Parameters.Routing.MIN_ROAD_LENGTH, DEFAULT_MIN_ROAD_LENGTH);
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        return runILS(from, to);
    }

    private Path runILS(int s, int d) {
        Route solution;
        if(shortestPath(s, d).getDistance() > MAX_COST) {
            solution = Route.newRoute(this, s, d);
        } else {
            solution = initialize(s, d);

            logger.info("Running ILS...");
            for(int i = 0; i < MAX_ITERATIONS; i++) {
                logger.info("Iteration " + i);
                List<Arc> arcs = solution.getCandidateArcsByIP();
                logger.info("Possible arcs to remove from solution: " + arcs.size());

                int randomIndex = random.nextInt(arcs.size());
                Arc e = arcs.remove(randomIndex);

                double b1 = (MAX_COST - solution.getCost()) + e.cost; // Remaining budget after removing e from solution

                Route path = generatePath(solution.getPrev(e).adjNode, solution.getNext(e).baseNode, b1, e.score, e.getCas());

                if(!path.isEmpty()) {
                    logger.info("Found path with with dist " + path.getCost());
                    int index = solution.removeArc(e);
                    solution.insertRoute(index, path);
                    for(Arc arc : solution.getArcs()) {
                        double b2 = (MAX_COST - solution.getCost()) + arc.cost; // Remaining budget after removing arc from solution

                        int startCAS = solution.getPrev(arc).adjNode;
                        int endCAS = solution.getNext(arc).baseNode;

                        Arc prev = solution.getArc(index);
                        Arc next = solution.getArc(index + 1);

                        if(path.contains(arc) || arc.equals(prev) || arc.equals(next)) {
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

        return solution.getPath(s, d, baseGraph, weighting);
    }

    private Route initialize(int s, int d) {
        Route route = Route.newRoute(this, s, d);
        // Add fake edge to start solution
        Arc arc = new Arc(-1, s, d, MAX_COST, 0);
        arc.setCas(computeCAS(null, s, d, MAX_COST));
        route.addArc(0, arc);

        return route;
    }

    private List<Arc> computeCAS(@Nullable List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        GHPoint focus1 = new GHPoint(nodeAccess.getLatitude(s), nodeAccess.getLongitude(s));
        GHPoint focus2 = new GHPoint(nodeAccess.getLatitude(d), nodeAccess.getLongitude(d));
        Shape ellipse = new Ellipse(focus1, focus2, cost);

        // If we don't have a CAS yet
        if(cas == null) {
            cas = getAllArcs(ellipse);
        }

        logger.info("Starting to compute CAS! num arcs: " + cas.size() + " cost: " + cost);

        outer:
        for(Arc arc : cas) {

            if(arc.score < MIN_ROAD_SCORE || arc.cost < MIN_ROAD_LENGTH) {
                continue;
            }

            for(GHPoint3D ghPoint3D : arc.getPoints()) {
                if(!ellipse.contains(ghPoint3D.lat, ghPoint3D.lon)) {
                    continue outer;
                }
            }

            if(getPathCost(s, d, arc) <= cost) {
                arc.qualityRatio = calcQualityRatio(s, d, arc);
                result.add(arc);
            }
        }

        logger.info("Finished computing CAS! size: " + result.size());

        return result;
    }

    private List<Arc> getAllArcs(final Shape shape) {
        logger.info("Fetching arcs from graph!");
        final List<Arc> arcs = new ArrayList<>();

        GHPoint center = shape.getCenter();
        QueryResult qr = locationIndex.findClosest(center.getLat(), center.getLon(), bikeEdgeFilter);
        // TODO: if there is no street close to the center it'll fail although there are roads covered. Maybe we should check edge points or some random points in the Shape instead?
        if(!qr.isValid())
            throw new IllegalArgumentException("Shape " + shape + " does not cover graph");

        if(shape.contains(qr.getSnappedPoint().lat, qr.getSnappedPoint().lon)) {
            arcs.add(getArc(qr.getClosestEdge()));
        }

        BreadthFirstSearch bfs = new BreadthFirstSearch() {
            final Shape localShape = shape;
            final GHBitSet edgeIds = new GHBitSetImpl();

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


        bfs.start(baseGraph.createEdgeExplorer(bikeEdgeFilter), qr.getClosestNode());

        logger.info("Got all arcs inside of ellipse! num: " + arcs.size());

        return arcs;
    }

    private Arc getArc(EdgeIteratorState edgeIterator) {
        int edge = edgeIterator.getEdge();
        int baseNode = edgeIterator.getBaseNode();
        int adjNode = edgeIterator.getAdjNode();
        double edgeCost = edgeIterator.getDistance();

        double edgeScore = bikePriorityWeighting
                .calcWeight(edgeIterator, false, baseNode);

        Arc arc = new Arc(edge, baseNode, adjNode, edgeCost, edgeScore);

        arc.setPoints(edgeIterator.fetchWayGeometry(0));
        return arc;
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

    private Route generatePath(int s, int d, double dist, double minProfit, List<Arc> cas) {
        logger.info("Generating path! dist: " + dist + " minProfit: " + minProfit + " cas size: " + cas.size());
        Route route = Route.newRoute(this, s, d);

        List<Arc> arcs = getCandidateArcsByQR(cas);
        while(!arcs.isEmpty() && route.getCost() < dist) {
            int randomIndex = random.nextInt(arcs.size());
            Arc e = arcs.remove(randomIndex);
            route.insertArcAtMinPathSegment(e, dist);
        }

        if(route.getScore() > minProfit) {
            return route;
        } else {
            return Route.newRoute(this, s, d);
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
