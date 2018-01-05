package com.graphhopper.routing.ils;

import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.PMap;
import com.graphhopper.util.StopWatch;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Random;

public class IteratedLocalSearch extends AbstractRoutingAlgorithm {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private Graph baseGraph;
    private Random random;

    private int numPaths = 10000;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public IteratedLocalSearch(Graph graph, Weighting weighting,
                               EdgeFilter levelEdgeFilter, PMap params) {
        super(graph, weighting, TraversalMode.EDGE_BASED_1DIR);

        baseGraph = graph.getBaseGraph();
        nodeAccess = baseGraph.getNodeAccess();
        this.levelEdgeFilter = levelEdgeFilter;
        random = new Random();

    }


    @Override
    public Path calcPath(int from, int to) {
        double sum = 0;

        logger.info("Starting CH profile....");
        for(int i = 0; i < numPaths; i++) {
            if(i % (numPaths / 20) == 0) {
                logger.info(Math.floor(((double) i / (double) numPaths) * 100) + "% complete!");
            }

            int maxNodeID = baseGraph.getNodes();

            int one = random.nextInt(maxNodeID);
            int two = random.nextInt(maxNodeID);

            while(one != two) {
                one = random.nextInt(maxNodeID);
                two = random.nextInt(maxNodeID);
            }

            final StopWatch timer = new StopWatch("ch-timer");
            timer.start();
            shortestPath(one, two);
            timer.stop();
            sum += timer.getTime();
        }

        double avg = sum / numPaths;
        logger.info("Average CH time: " + avg + " ms");

        return null;
    }

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
        boolean isFinished = false;
        return isFinished;
    }

    // Unused
    @Override
    protected Path extractPath() {
        return null;
    }
}
