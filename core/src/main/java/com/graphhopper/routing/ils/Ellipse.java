package com.graphhopper.routing.ils;

import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.Helper;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.Shape;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

/**
 * Class which represents an Ellipse on the map. Used by the {@link IteratedLocalSearch} algorithm for restricting
 * the search space.
 * <p>
 * Note: This does not fully implement the Shape interface!
 */
class Ellipse implements Shape {

    private static DistanceCalc calc = Helper.DIST_EARTH;

    private GHPoint focus1;
    private GHPoint focus2;
    private double radius;

    Ellipse(GHPoint focus1, GHPoint focus2, double radius) {
        this.focus1 = focus1;
        this.focus2 = focus2;
        this.radius = radius;
    }

    @Override
    public boolean intersect(Shape o) {
        throw new NotImplementedException();
    }

    @Override
    public boolean contains(double lat, double lon) {
        return calc.calcDist(lat, lon, focus1.lat, focus1.lon) +
                calc.calcDist(lat, lon, focus2.lat, focus2.lon) <= radius;
    }

    @Override
    public boolean contains(Shape s) {
        throw new NotImplementedException();
    }

    @Override
    public BBox getBounds() {
        throw new NotImplementedException();
    }

    @Override
    public GHPoint getCenter() {
        GHPoint point = new GHPoint();

        double dLon = Math.toRadians(focus2.lon - focus1.lon);

        //convert to radians
        double lat1 = Math.toRadians(focus1.lat);
        double lat2 = Math.toRadians(focus2.lat);
        double lon1 = Math.toRadians(focus1.lon);

        double Bx = Math.cos(lat2) * Math.cos(dLon);
        double By = Math.cos(lat2) * Math.sin(dLon);
        double lat3 = Math.atan2(Math.sin(lat1) + Math.sin(lat2), Math.sqrt((Math.cos(lat1) + Bx) * (Math.cos(lat1) + Bx) + By * By));
        double lon3 = lon1 + Math.atan2(By, Math.cos(lat1) + Bx);

        point.lat = Math.toDegrees(lat3);
        point.lon = Math.toDegrees(lon3);
        return point;
    }


    @Override
    public double calculateArea() {
        throw new NotImplementedException();
    }
}
