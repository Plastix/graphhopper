package com.graphhopper.routing.ils;

import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.Helper;
import com.graphhopper.util.shapes.GHPoint;

public class Ellipse {

    private DistanceCalc calc = Helper.DIST_EARTH;
    private GHPoint focus1;
    private GHPoint focus2;
    private double radius;

    Ellipse(GHPoint focus1, GHPoint focus2, double radius) {
        this.focus1 = focus1;
        this.focus2 = focus2;
        this.radius = radius;
    }

    public boolean contains(double lat, double lon) {
        return calc.calcNormalizedDist(lat, lon, focus1.lat, focus1.lon) +
                calc.calcNormalizedDist(lat, lon, focus2.lat, focus2.lon) <= radius;
    }
}
