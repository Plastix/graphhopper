package com.graphhopper.routing.ils;

import com.graphhopper.routing.Path;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

import java.util.ArrayList;
import java.util.List;

/**
 * Object which represents a path created by the {@link IteratedLocalSearch}
 * algorithm.
 */
final class Route {

    private ShortestPathCalculator sp;
    private List<Arc> arcs;
    private List<Double> blankSegments;
    private double cost, score;

    private Route(ShortestPathCalculator shortestPathCalculator) {
        sp = shortestPathCalculator;
        arcs = new ArrayList<>();
        blankSegments = new ArrayList<>();
    }

    public static Route newRoute(ShortestPathCalculator shortestPathCalculator) {
        return new Route(shortestPathCalculator);
    }

    void addArc(int index, Arc arc) {
        int length = getNumArcs();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException();
        }

        if(index == 0) {
            // Add arc to beginning of path: only requires inserting one blank path segment
            // 1-2-3 --> 0-1-2-3
            double distance = sp.shortestPath(arc.adjNode, arcs.get(0).baseNode).getDistance();
            blankSegments.add(0, distance);
            cost += distance;

        } else if(index == length) {
            // Add arc to end of path: only requires inserting one blank path segment
            // 1-2-3 --> 1-2-3-4
            double distance = sp.shortestPath(arcs.get(length - 1).adjNode, arc.baseNode).getDistance();
            blankSegments.add(distance);
            cost += distance;
        } else {
            // Insert arc into middle of path
            // Requires removing a blank path segment and creating two new ones
            // 1-2-3 --> 1-4-2-3

            // Guaranteed to not cause IOB
            Arc prevArc = arcs.get(index - 1);
            Arc nextArc = arcs.get(index + 1);

            double segment1 = sp.shortestPath(prevArc.adjNode, arc.baseNode).getDistance();
            cost += segment1;
            double segment2 = sp.shortestPath(arc.adjNode, nextArc.baseNode).getDistance();
            cost += segment2;

            double removed = blankSegments.remove(index - 1);
            cost -= removed;

            blankSegments.add(index - 1, segment2);
            blankSegments.add(index - 1, segment1);

        }

        arcs.add(index, arc);
        cost += arc.cost;
        score += arc.score;
    }

    public void removeArc(Arc a) {
        int index = arcs.indexOf(a);

        if(index != -1) {
            int length = getNumArcs();
            Arc arc = arcs.get(index);

            if(index == 0) {
                blankSegments.remove(0);
            } else if(index == length - 1) {
                // For n arcs there are n-1 blank path segments
                blankSegments.remove(index - 1);
            } else {
                Arc prevArc = arcs.get(index - 1);
                Arc nextArc = arcs.get(index + 1);

                double segment = sp.shortestPath(prevArc.adjNode, nextArc.baseNode).getDistance();

                double cost1 = blankSegments.remove(index - 1);
                double cost2 = blankSegments.remove(index - 1);

                cost -= cost1;
                cost -= cost2;

                blankSegments.add(index - 1, segment);
                cost += segment;
            }

            arcs.remove(index);
            cost -= arc.cost;
            score -= arc.score;
        }
    }

    public void insertRoute(int index, Route route) {
        throw new NotImplementedException();
    }

    public double getCost() {
        return cost;
    }

    public double getScore() {
        return score;
    }

    // TODO (Aidan)
    public Path getPath() {
        throw new NotImplementedException();
    }

    public int getNumArcs() {
        return arcs.size();
    }

    public List<Arc> getArcs() {
        return arcs;
    }

    public Arc getPrev(Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index - 1 >= 0) ? arcs.get(index - 1) : a;
    }

    public Arc getNext(Arc a) {
        int index = arcs.indexOf(a);

        return (index != -1 && index + 1 <= getNumArcs() - 1) ? arcs.get(index + 1) : a;
    }

    public boolean contains(Arc a) {
        return arcs.contains(a);
    }

    public Segment getSmallestSegment() {
        int index = -1;
        double min = Double.MAX_VALUE;

        for(int i = 0; i < blankSegments.size(); i++) {
            double value = blankSegments.get(i);
            if(value < min) {
                min = value;
                index = i;
            }
        }

        int arcIndex = index + 1;
        int start = arcs.get(arcIndex - 1).adjNode;
        int end = arcs.get(arcIndex + 1).baseNode;

        return new Segment(start, end, arcIndex, min);
    }

    public class Segment {
        final int startNode, endNode, arcIndex;
        final double cost;

        Segment(int startNode, int endNode, int arcIndex, double cost) {
            this.startNode = startNode;
            this.endNode = endNode;
            this.arcIndex = arcIndex;
            this.cost = cost;
        }
    }
}
