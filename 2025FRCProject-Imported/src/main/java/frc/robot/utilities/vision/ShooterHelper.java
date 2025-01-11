package frc.robot.utilities.vision;

import java.lang.reflect.Array;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterHelper {
public static double[] coordinates(double theta, double distance){
  double x = Math.cos(theta)*distance;
  double y = Math.sin(theta)*distance;
return new double [] {x,y};
}
}
