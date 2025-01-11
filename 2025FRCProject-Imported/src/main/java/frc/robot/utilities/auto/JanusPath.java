package frc.robot.utilities.auto;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class JanusPath {

    private ArrayList<JanusWaypoint> waypoints;
    private boolean isCommand = false;
    private JanusConfig config;
    private ArrayList<JanusState> states;

    public JanusPath(ArrayList<JanusWaypoint> waypoints, JanusConfig config){
        this.waypoints = waypoints;
        this.config = config;
    }

    public JanusPath(JanusWaypoint waypoint, JanusConfig config){
        this(new ArrayList<>() {{add(waypoint);}}, config);
        isCommand = true;
    }

    public void calculatePath(){
        states = calculateOptimalSpeedsAndTime(waypoints, config);

        for (JanusState js : states) {
            // System.out.println(js.toString());
        }

        states = calculateAcclerationRamps(states);

        Collections.reverse(states);
        for (JanusState js : states) {
            // System.out.println(js.toString());
        }
        
    }

    public ArrayList<JanusState> calculateOptimalSpeedsAndTime(ArrayList<JanusWaypoint> waypoints, JanusConfig config){
        ArrayList<JanusState> states = new ArrayList<>();
        for (int i = 0; i < waypoints.size()-1; i++) {
            JanusWaypoint start = waypoints.get(i);
            JanusWaypoint end = waypoints.size() <= i+1 ? start : waypoints.get(i+1);
            states.add(calculateState(start, end));
        }
        states.add(JanusState.empty());
        return states;
    }

    public JanusState calculateState(JanusWaypoint start, JanusWaypoint end){
        double angle = start.angleFrom(end);
        JanusVector d = JanusVector.fromResultant(start.distanceFrom(end), angle);
        JanusVector a = JanusVector.fromResultant(config.maxAccelerationMeters(), angle);

        JanusVector vf = JanusVector.fromResultant(Math.pow(2*a.resultant()*d.resultant(), 0.5), angle);
        JanusComponent xComp = new JanusComponent(0, 0, d.xComp(), a.xComp(), 0, 0, vf.xComp());
        JanusComponent yComp = new JanusComponent(0, 0, d.yComp(), a.yComp(), 0, 0, vf.yComp());


        double thetaD = end.getTheta() - start.getTheta();
        double thetaA = config.maxAngularAccelerationMeters();
        double thetaVf = Math.pow(2*thetaA*thetaD, 0.5);

        JanusComponent thetaComp = new JanusComponent(0, 0, thetaD, thetaA, 0, 0, thetaVf);

        return new JanusState(xComp, yComp, thetaComp, angle);
    }

    public JanusState recalculateState(JanusComponent xComp, JanusComponent yComp, JanusComponent thetaComp, double xVi, double yVi, double thetaVi, double angle){
        xComp.vi = xVi;
        yComp.vi = yVi;
        thetaComp.vi = thetaVi;

        double vi = JanusVector.getResultant(xVi, yVi);
        double a = JanusVector.getResultant(xComp.a, yComp.a);
        double d = JanusVector.getResultant(xComp.d, yComp.d);

        JanusVector vf = JanusVector.fromResultant(Math.pow(Math.pow(vi,2) + 2*a*d,0.5), angle);
        xComp.vf = vf.xComp();
        yComp.vf = vf.yComp();
        // System.out.println(vf + " " + angle);

        double thetaVf = Math.pow(Math.pow(thetaVi,2) + 2*thetaComp.a*thetaComp.d, 0.5);
        thetaComp.vf = thetaVf;
        return new JanusState(xComp, yComp, thetaComp, angle);
    }

    public JanusComponent timeStretch(JanusComponent comp, double time){
        double t = time - comp.timestamp;
        // double vf = ((2*comp.d)/t ) - comp.vi;
       // comp.vf = vf;
        comp.t = t;
        return comp;
    }

    public ArrayList<JanusState> calculateAcclerationRamps(ArrayList<JanusState> states){
        ArrayList<JanusState> modifiedStates = new ArrayList<>();
        double timestamp = 0;

        Pose2d pose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        for (int i = 0; i < states.size(); i++) {
            JanusState start = states.get(i);
            JanusState end = states.size() <= i+1 ? JanusState.empty() : states.get(i+1);
            ArrayList<JanusComponent> xStates = getPathComponentValues(start.xComps().get(0), end.xComps().get(0));
            ArrayList<JanusComponent> yStates = getPathComponentValues(start.yComps().get(0), end.yComps().get(0));
            ArrayList<JanusComponent> thetaStates = getPathComponentValues(start.thetaComps().get(0), end.thetaComps().get(0));

            double tx = 0;
            double ty = 0;
            double tt = 0;

            double totatT = 0;
            for (int j = 0; j < xStates.size(); j++) {
                JanusComponent state = xStates.get(j);
                state.timestamp = tx;
                tx += state.t;
            }

            for (int j = 0; j < yStates.size(); j++) {
                JanusComponent state = yStates.get(j);
                state.timestamp = ty;
                ty += state.t;
            }

            for (int j = 0; j < thetaStates.size(); j++) {
                JanusComponent state = thetaStates.get(j);
                state.timestamp = tt;
                tt += state.t;
            }

            totatT = tx > ty ? tx : ty;
            totatT = tt > totatT ? tt : totatT;
            // System.out.println(yStates.get(yStates.size()-1));
            // xStates.set(xStates.size()-1, timeStretch(xStates.get(xStates.size()-1), totatT));
            // yStates.set(yStates.size()-1, timeStretch(yStates.get(yStates.size()-1), totatT));
            // thetaStates.set(thetaStates.size()-1, timeStretch(thetaStates.get(thetaStates.size()-1), totatT));
            // System.out.println(yStates.get(yStates.size()-1));

            double x = 0;
            double y = 0;
            double theta = 0;

            for (JanusComponent state : xStates) {
                state.a = solveAcceleration(state.d, state.vi, state.t);
                state.startingPose = x;
                x += state.d;
            }

            for (JanusComponent state : yStates) {
                state.a = solveAcceleration(state.d, state.vi, state.t);
                state.startingPose = y;
                y += state.d;
            }

            for (JanusComponent state : thetaStates) {
                state.a = solveAcceleration(state.d, state.vi, state.t);
                state.startingPose = theta;
                theta += state.d;
            }

            

            int index = states.indexOf(end);
            if(index >= 0){
                JanusState state = recalculateState(end.xComps().get(0), end.yComps().get(0), end.thetaComps().get(0), xStates.get(xStates.size()-1).vf, yStates.get(yStates.size()-1).vf, thetaStates.get(thetaStates.size()-1).vf, end.angle());
                states.set(index, state);
            }
            
            xStates.add(JanusComponent.empty(x, tx));
            
            yStates.add(JanusComponent.empty(y, ty));

            thetaStates.add(JanusComponent.empty(theta, tt));
            // System.out.println(theta);
            Collections.reverse(xStates);
            Collections.reverse(yStates);
            Collections.reverse(thetaStates);

            modifiedStates.add(new JanusState(pose, timestamp, xStates, yStates, thetaStates, start.angle()));
            timestamp += totatT;
            pose = new Pose2d(pose.getX()+x, pose.getY()+y, Rotation2d.fromRadians(pose.getRotation().getRadians()+theta));
        }
        return modifiedStates;
    }

    int isX = 0;
    public ArrayList<JanusComponent> getPathComponentValues(JanusComponent start, JanusComponent end){
        // String title = (isX == 0) ? "X" : (isX == 1) ? "Y" : "R";
        // System.out.println("======="+title+"=======");
        isX = isX >= 3 ? 0 : isX+1;
        ArrayList<JanusComponent> states = new ArrayList<>();
        double m1, m2;
        double b1, b2;
        // System.out.println("a:"+start.a);
        // System.out.println("vf:"+start.vf);
        // System.out.println("vi:"+start.vi);
        double t1 = Math.abs((start.vf - start.vi) / start.a);

        m1 = calculateSlope(0.0, t1, start.vi, start.vf);
        b1 = calculateSlopeYIntercept(0.0, start.vi, m1);

        double endvf = (start.vf * end.vf <= 0) ? 0 : end.vf;
        double t2 = Math.abs((endvf - start.vf) / start.a);
        double t3 = calculateTime(start.d, start.a, start.vi);
        // System.out.println("endvf:"+endvf);
        // System.out.println("t1:"+t1);
        // System.out.println("t2:"+t2);
        // System.out.println("t3:"+t3);

        m2 = calculateSlope(t3-t2, t3, start.vf, endvf);
        b2 = calculateSlopeYIntercept(t3-t2,start.vf, m2);

        //System.out.println(t2);

        double xInt = calculateLineXIntercept(m1, m2, b1, b2);
        //xInt += start.d;
        double yInt = calculateLineYIntercept(m1, b1, xInt);
        yInt = Double.isNaN(yInt) ? 0 : yInt;

        // System.out.println( "y="+m1 + "(x)+"+ b1 + "|y="+m2 + "(x)+"+ b2);

        // System.out.println("yint:"+yInt);

        if(distanceAtMaxSpeed(start.vi, yInt, xInt)  > Math.abs(start.d)){
            double vf2 = Math.pow(start.vi, 2) + 2*m1*start.d;
            yInt = Math.pow(vf2, 0.5);
           
        }
        // System.out.println("yint:"+yInt);
        if(Math.abs(yInt) >= Math.abs(start.vf)){
            yInt = start.vf;
        }

        if(Math.abs(endvf) >= Math.abs(yInt)){
            endvf = yInt;
        }
        // System.out.println("endvf:"+endvf);
        // System.out.println("yint:"+yInt);

        JanusComponent forward;
        JanusComponent sustained;
        JanusComponent reverse;

        double tf = Math.abs(timeAtMaxSpeed(yInt, start.a, start.vi));
        // System.out.println("tf:"+tf);
        // System.out.println("a:"+start.a);
        // System.out.println("vi:"+start.vi);
        // System.out.println("d:"+start.d);
        double df = distanceAtMaxSpeed(start.vi, yInt, tf);
        if(df != 0){
            yInt = calculateFinalVelocity(start.vi, df, tf);
        }
        forward = new JanusComponent(0, 0, df, 0, tf, start.vi, yInt);
        states.add(forward);
        // System.out.println("yint:" +yInt);
        double ds = start.d - df;
        // System.out.println("ds:" +ds);
        
        if(Math.abs(ds) > 0.001){
            double tr = Math.abs(timeAtMaxSpeed(endvf, start.a, yInt));
            double dr = distanceAtMaxSpeed(yInt, endvf, tr);
            reverse = new JanusComponent(0, 0, dr, 0, tr, yInt, endvf);
            
            ds -= dr;

            if(Math.abs(ds) > 0.001){
                double ts = Math.abs(solveTime(ds, yInt, yInt));
                sustained = new JanusComponent(0, 0, ds, Double.NaN, ts, yInt, yInt);
                states.add(sustained);
            }
            
            states.add(reverse);
        }        
        
        return states;
    }

    private double calculateLineXIntercept(double m1, double m2, double b1, double b2){
        return ((b2-b1)/(m1-m2));
    }

    private double calculateLineYIntercept(double m1, double b1, double x){
        return m1*x+b1;
    }

    private double calculateSlope(double x1, double x2, double y1, double y2){
        double value = (y2-y1)/(x2-x1);
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateSlopeYIntercept(double x, double y, double m){
        double value = y-(m*x); 
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateFinalVelocity(double vi, double d, double t){
        double value = (2*d)/t - vi;
        return Double.isNaN(value) ? 0 : value;
    }

    public double solveAcceleration(double d, double vi, double t){
        double value = (2*(d-vi*t))/(t*t);
        return Double.isNaN(value) ? 0 : value;

    }

    private double timeAtMaxSpeed(double vf, double a, double vi){
        double val = (vf - vi)/a;
        return Double.isNaN(val) ? 0 : val;
    }

    private double distanceAtMaxSpeed(double vi, double vf, double t){
        double val = (vi + vf)/2 * t;
        return Double.isNaN(val) ? 0 : val;
    }

    public boolean endOfPath(Pose2d pose){
        if(isCommand) return waypoints.get(0).getCommand().isFinished();
    
        // Check if the list of waypoints is empty or if there's only a command without a path
        if(waypoints.isEmpty() || (waypoints.size() == 1 && isCommand)) return true;
    
        // Get the last waypoint in the list
        JanusWaypoint lastWaypoint = waypoints.get(waypoints.size() - 1);
    
        // Check if the last waypoint is a command that should keep moving
        if(lastWaypoint.isCommand() && lastWaypoint.keepMoving) return false;
    
        // Calculate the distance and angle difference between the current pose and the last waypoint's pose
        Pose2d lastWaypointPose = new Pose2d(lastWaypoint.getX(), lastWaypoint.getY(), new Rotation2d(lastWaypoint.getTheta()));
        double distance = lastWaypointPose.getTranslation().getDistance(pose.getTranslation());
        double angleDifference = Math.abs(lastWaypointPose.getRotation().getDegrees() - pose.getRotation().getDegrees());
    
        // Define tolerances for when the robot is considered to have reached the end of the path
        final double distanceTolerance = 0.1; // meters
        final double angleTolerance = 5.0; // degrees
    
        // Determine if the robot is within tolerances of the last waypoint
        return distance <= distanceTolerance && angleDifference <= angleTolerance;
    }
    

    private JanusState findRelaventState(double time){
        for (int i = 0; i < states.size(); i++) {
            JanusState state = states.get(i);
            if(state.timestamp() > time) continue;
            return state;
        }
        return states.get(states.size()-1);
    }

    public ChassisSpeeds getSpeedsAtTime(double time){
        return findRelaventState(time).getChassisSpeedsAtTime(time);
    }

    public void runCommand(){
        if(isCommand()) waypoints.get(0).getCommand().schedule();
    }

    public boolean isCommand(){
        return isCommand;
    }

    public Pose2d getPoseAtTime(double time){
        return findRelaventState(time).getPoseAtTime(time);
    }

    private double solveTime(double d, double vi, double vf){
        double value =  d /((vi+vf)/2);
        return Double.isNaN(value) ? 0 : value;
    }

    private double calculateTime(double d, double a, double vi){
        double root1, root2, root;
        double determinant = vi * vi - 4 * a * -d;
        if (determinant == 0.0){
            root = -vi/(2.0*a);
           
        }else if(determinant > 0){
            root1 = (-vi + Math.sqrt(determinant)) / (2.0 * a);
            root2 = (-vi - Math.sqrt(determinant)) / (2.0 * a);
            root = root1 > root2 ? root1 : root2;
        }else{
            root = 0;
        }
        return Double.isNaN(root) ? 0 : root; 
    }
}
