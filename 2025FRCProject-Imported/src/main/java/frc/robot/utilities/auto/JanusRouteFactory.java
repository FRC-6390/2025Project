package frc.robot.utilities.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public class JanusRouteFactory {

    private JanusConfig config;
    private ArrayList<JanusWaypoint> route = new ArrayList<>();

    public JanusRouteFactory(JanusConfig config) {
        this.config = config;
        route.add(new JanusWaypoint(0, 0, 0));
    }

    public JanusRouteFactory to(double x, double y) {
        return to(x, y, Double.NEGATIVE_INFINITY);
    }

    public JanusRouteFactory to(double x, double y, double theta) {
        route.add(new JanusWaypoint(x, y, theta));
        return this;
    }

    public JanusRouteFactory run(Command command) {
        return run(command, false);
    }

    public JanusRouteFactory run(Command command, boolean keepMoving) {
        route.add(new JanusWaypoint(command, keepMoving));
        return this;
    }

    public JanusRoute build() {
        correctWaypoints(route);
        ArrayList<JanusPath> path = createPath(route);
        return new JanusRoute(path, config);
    }

    private void correctWaypoints(ArrayList<JanusWaypoint> waypoints) {
        for (int i = 0; i < waypoints.size(); i++) {
            JanusWaypoint point = waypoints.get(i);
            // skips Commands
           
            if (point.isCommand()) {
                JanusWaypoint previousPoint = getPreviousWaypoint(i, waypoints);
                if (previousPoint != null)
                    point.setPose(previousPoint);
                else
                    point.setPose(0, 0, 0);
                continue;
            }
            // adds theta to any points missing it
            if (Double.isInfinite(point.getTheta())) {
                JanusWaypoint previousPoint = getPreviousWaypoint((i - 1), waypoints);
                if (previousPoint != null)
                    point.setTheta(previousPoint.getTheta());
                else
                    point.setTheta(0);
            }else{
                point.setTheta(Math.IEEEremainder(Math.toRadians(point.getTheta()), 2 * Math.PI));
            }
        }
    }

    // tbh never thought I would ever use recursion, Ms.P was right
    private JanusWaypoint getPreviousWaypoint(int id, ArrayList<JanusWaypoint> waypoints) {
        if (id <= 0) // Adjusted to <= to include the first element check
            return null;
        id--; // Move to the previous element right away
        JanusWaypoint point = waypoints.get(id);
        if (point.isCommand()) {
            return getPreviousWaypoint(id, waypoints); // Now correctly moves to the previous index
        } else {
            return point;
        }
    }

    // takes the inputed points and splits them into section, commands are the
    // dividing point
    // Also converts them into JanusPath to have the path calulated
    private ArrayList<JanusPath> createPath(ArrayList<JanusWaypoint> waypoints) {
        ArrayList<JanusPath> paths = new ArrayList<>();

        ArrayList<JanusWaypoint> section = new ArrayList<>();
        for (int i = 0; i < waypoints.size(); i++) {
            JanusWaypoint point = waypoints.get(i);
            if (point.isCommand()) {
                if (section.size() > 0)
                    paths.add(new JanusPath(section, config));
                section.clear();
                paths.add(new JanusPath(point, config));
                continue;
            }
            section.add(point);
        }

        paths.add(new JanusPath(section, config));
        return paths;

    }

}
