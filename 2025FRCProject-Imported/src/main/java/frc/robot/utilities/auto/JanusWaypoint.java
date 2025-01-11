package frc.robot.utilities.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class JanusWaypoint {

    enum JanusSpline {
        Cubic,

    }
    
    private double x, y, theta;
    private Command command;
    public boolean keepMoving;

    public JanusWaypoint(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public JanusWaypoint(Command command, boolean keepMoving){
        this.command = command;
        this.keepMoving = keepMoving;
    }

    public JanusWaypoint toNewPostion(double x, double y){
        return new JanusWaypoint(x, y, theta);
    }

    public double getTheta() {
        return theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setTheta(double theta) {
        this.theta = Math.atan2(Math.sin(theta), Math.cos(theta));
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Command getCommand() {
        return command;
    }

    public boolean isCommand(){
        return command != null;
    }

    public void setPose(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void setPose(JanusWaypoint waypoint){
        setPose(waypoint.getX(), waypoint.getY(), waypoint.getTheta());
    }

    public double distanceFrom(JanusWaypoint point){
        return Math.sqrt(Math.pow((point.getY() - getY()),2) + Math.pow((point.getX() - getX()),2));
    }

    public double angleFrom(JanusWaypoint point){
        return Math.atan2((point.getY() - getY()),  (point.getX() - getX()));
    }

}