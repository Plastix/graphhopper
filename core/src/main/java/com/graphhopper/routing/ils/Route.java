package com.graphhopper.routing.ils;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * Object which represents a path created by the {@link IteratedLocalSearch}
 * algorithm.
 */
final class Route {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    private ShortestPathCalculator sp;
    private int s, d;
    private List<Arc> arcs;
    private List<Double> blankSegments;
    private double startSegment, endSegment;
    private double cost, score;

    private Route(ShortestPathCalculator shortestPathCalculator, int s, int d) {
        sp = shortestPathCalculator;
        arcs = new ArrayList<>();
        blankSegments = new ArrayList<>();
        cost = 0;
        score = 0;
        this.s = s;
        this.d = d;
        startSegment = 0;
        endSegment = 0;
    }

    public static Route newRoute(ShortestPathCalculator sp, int s, int d) {
        return new Route(sp, s, d);
    }

    void addArc(int index, Arc arc) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        if(index == 0) {
            addLeadingPathSegment(arc);
            updateStartPathSegment(arc);
        }

        if(index == length) {
            addTrailingPathSegment(arc);
            updateEndPathSegment(arc);
        }

        if(index > 0 && index < length) {
            updateBorderingPathSegments(index, arc, arc);
        }

        arcs.add(index, arc);
        cost += arc.cost;
        score += arc.score;
    }

    public int removeArc(Arc a) {
        int index = arcs.indexOf(a);
        int length = getNumArcs();

        if(index != -1) {
            // We only need to remove blank path segments if we have more than one arc
            if(length > 1) {
                if(index == 0) {
                    // Remove head arc: remove one blank path segment
                    // 1-2-3 --> 2-3
                    double segment = blankSegments.remove(index);
                    cost -= segment;
                } else if(index == length - 1) {
                    // Remove tail arc: remove one blank path segment
                    // 1-2-3 --> 1-2
                    //
                    // For n arcs there are n-1 blank path segments
                    double segment = blankSegments.remove(index - 1);
                    cost -= segment;

                } else {
                    // Remove middle arc: remove two blank path segments and add a new one
                    // 1-2-3 --> 1-3
                    Arc prevArc = arcs.get(index - 1);
                    Arc nextArc = arcs.get(index + 1);

                    double cost1 = blankSegments.remove(index - 1);
                    double cost2 = blankSegments.remove(index - 1);

                    cost -= cost1;
                    cost -= cost2;

                    double segment = sp.shortestPath(prevArc.adjNode, nextArc.baseNode).getDistance();
                    blankSegments.add(index - 1, segment);
                    cost += segment;
                }
            }

            if(index == 0) {
                Arc next = length > 1 ? arcs.get(1) : null;
                updateStartPathSegment(next);
            }

            if(index == length - 1) {
                Arc prev = length > 1 ? arcs.get(length - 2) : null;
                updateEndPathSegment(prev);
            }

            arcs.remove(index);
            cost -= a.cost;
            score -= a.score;
        }

        return index;
    }

    public void insertRoute(int index, Route route) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        if(!route.isEmpty()) {
            Arc first = route.arcs.get(0);
            Arc last = route.arcs.get(route.getNumArcs() - 1);

            if(index == 0) {
                addLeadingPathSegment(last);
                updateStartPathSegment(first);
            }

            if(index == length) {
                addLeadingPathSegment(first);
                updateEndPathSegment(last);
            }

            if(index > 0 && index < length) {
                updateBorderingPathSegments(index, first, last);
            }

            // We need to remove the inserted routes starting and ending path cost
            // We recalculate the new path segments below
            route.cost -= route.startSegment;
            route.cost -= route.endSegment;

            score += route.score;
            cost += route.cost;
            arcs.addAll(index, route.arcs);
            blankSegments.addAll(index, route.blankSegments);
        }
    }

    private void addLeadingPathSegment(Arc arc) {
        if(!isEmpty()) {
            // Add arc to beginning of path: only requires inserting one blank path segment
            // 1-2-3 --> 0-1-2-3
            double distance = sp.shortestPath(arc.adjNode, arcs.get(0).baseNode).getDistance();
            blankSegments.add(0, distance);
            cost += distance;
        }
    }

    private void addTrailingPathSegment(Arc arc) {
        if(!isEmpty()) {
            // Add arc to end of path: only requires inserting one blank path segment
            // 1-2-3 --> 1-2-3-4
            double distance = sp.shortestPath(arcs.get(getNumArcs() - 1).adjNode, arc.baseNode).getDistance();
            blankSegments.add(distance);
            cost += distance;
        }
    }

    private void updateBorderingPathSegments(int index, Arc left, Arc right) {
        if(index < 1 || index > getNumArcs() - 1) {
            throw new IndexOutOfBoundsException(String.valueOf(index));
        }

        if(!isEmpty()) {
            // Insert arc into middle of path
            // Requires removing a blank path segment and creating two new ones
            // 1-2-3 --> 1-4-2-3

            // Guaranteed to not cause IOB because index >= 1 && <= length-1
            Arc prevArc = arcs.get(index - 1);
            Arc nextArc = arcs.get(index);

            double segment1 = sp.shortestPath(prevArc.adjNode, left.baseNode).getDistance();
            cost += segment1;
            double segment2 = sp.shortestPath(right.adjNode, nextArc.baseNode).getDistance();
            cost += segment2;

            double removed = blankSegments.remove(index - 1);
            cost -= removed;

            blankSegments.add(index - 1, segment2);
            blankSegments.add(index - 1, segment1);
        }
    }

    private void updateStartPathSegment(Arc arc) {
        // Edge case
        // If our arc's starting node is not our start of route we need to add another path segment
        cost -= startSegment;
        if(arc != null && arc.baseNode != s) {
            startSegment = sp.shortestPath(s, arc.baseNode).getDistance();
        } else {
            startSegment = 0;
        }
        cost += startSegment;
    }

    private void updateEndPathSegment(Arc arc) {
        // Edge case
        // If our arc's ending node is not our end of route we need to add another path segment
        cost -= endSegment;
        if(arc != null && arc.adjNode != d) {
            endSegment = sp.shortestPath(arc.adjNode, d).getDistance();
        } else {
            endSegment = 0;
        }
        cost += endSegment;
    }

    public double getCost() {
        return cost;
    }

    public double getScore() {
        return score;
    }

    public Path getPath(int s, int d, Graph graph, Weighting weighting) {
        logger.info("Route cost: " + getCost());
        Path path = new Path(graph, weighting);

        Arc temp = null;
        for(int i = 0; i < arcs.size(); i++) {

            Arc arc = arcs.get(i);

            // Add leading gap edges (s --> a_0)
            if(i == 0 && arc.baseNode != s) {
                for(EdgeIteratorState edge : sp.shortestPath(s, arc.baseNode).calcEdges()) {
                    path.processEdge(edge.getEdge(), edge.getAdjNode(), edge.getEdge());
                }
            }

            // Fill in gap between two arcs (a_i --> a_i+1)
            if(temp != null && temp.adjNode != arc.baseNode) {
                for(EdgeIteratorState edge : sp.shortestPath(temp.adjNode, arc.baseNode).calcEdges()) {
                    path.processEdge(edge.getEdge(), edge.getAdjNode(), edge.getEdge());
                }
            }

            path.processEdge(arc.edgeId, arc.adjNode, arc.edgeId);

            // Add trailing gap edges (a_n --> d)
            if(i == arcs.size() - 1 && arc.adjNode != d) {
                for(EdgeIteratorState edge : sp.shortestPath(arc.adjNode, d).calcEdges()) {
                    path.processEdge(edge.getEdge(), edge.getAdjNode(), edge.getEdge());
                }
            }

            temp = arc;
        }

        return path
                .setEndNode(d)
                .setFromNode(s)
                .setFound(!isEmpty());
    }

    private int getNumArcs() {
        return arcs.size();
    }

    public boolean isEmpty() {
        return getNumArcs() == 0;
    }

    public List<Arc> getArcs() {
        return arcs;
    }

    public List<Arc> getCandidateArcsByIP() {
        List<Arc> result = new ArrayList<>();
        double avgIP = 0;
        for(Arc ca : arcs) {
            ca.improvePotential = calcImprovePotential(ca);
            avgIP += ca.improvePotential;
        }
        avgIP /= arcs.size();

        for(Arc ca : arcs) {
            if(ca.improvePotential >= avgIP) {
                result.add(ca);
            }
        }

        return result;
    }

    private double calcImprovePotential(Arc arc) {
        int v1 = getPrev(arc).adjNode;
        int v2 = getNext(arc).baseNode;

        double score = 0;
        double maxDist = 0;

        double dist = sp.getPathCost(v1, v2, arc);

        for(Arc e : arc.getCas()) {
            score += e.score - arc.score;
            maxDist = Math.max(maxDist, sp.getPathCost(v1, v2, e));
        }

        return score / (maxDist - dist);
    }

    public Arc getPrev(Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index - 1 >= 0) ? arcs.get(index - 1) : a;
    }

    public Arc getNext(Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index + 1 <= getNumArcs() - 1) ? arcs.get(index + 1) : a;
    }

    public Arc getArc(int index) {
        if(index >= 0 && index < getNumArcs()) {
            return arcs.get(index);
        }

        return null;
    }

    public boolean contains(Arc a) {
        return arcs.contains(a);
    }

    public void insertArcAtMinPathSegment(Arc arc, double budget) {
        int pathIndex = -1;
        double minPathValue = Double.MAX_VALUE;

        if(getNumArcs() > 1) {
            // We have at least 2 arcs and at least 1 blank path segment
            // Find smallest blank path segment
            for(int i = 0; i < blankSegments.size(); i++) {
                double value = blankSegments.get(i);
                if(value < minPathValue) {
                    minPathValue = value;
                    pathIndex = i;
                }
            }

            int arcIndex = pathIndex + 1;
            int start = (arcIndex - 1 >= 0) ? arcs.get(arcIndex - 1).adjNode : arcs.get(arcIndex).adjNode;
            int end = arcs.get(arcIndex).baseNode;

            if(sp.getPathCost(start, end, arc) <=
                    budget - getCost() + minPathValue) {
                addArc(arcIndex, arc);
            }

            // If we have 0 or 1 arcs we have no blank path segments to check
        } else if(getNumArcs() == 1) {
            if(arc.cost + sp.shortestPath(arc.adjNode, arcs.get(0).baseNode).getDistance() <= budget - getCost()) {
                addArc(0, arc);
            }
        } else if(arc.cost <= budget - getCost()) {
            addArc(0, arc);
        }


    }
}
