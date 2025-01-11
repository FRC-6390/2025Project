package frc.robot.utilities.auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utilities.controlloop.PID;

public class JanusRoute {

    private ArrayList<JanusPath> path = new ArrayList<>();
    private int section;
    private JanusPath currentSection;
    private JanusConfig config;
    private PID xPID, yPID, thetaPID;
    private Timer timer;
    private Supplier<Pose2d> odometry;

    public JanusRoute(ArrayList<JanusPath> path, JanusConfig config) {
        this.path = path;
        this.config = config;
        xPID = new PID(this.config.xyPIDConfig());
        yPID = new PID(this.config.xyPIDConfig());
        thetaPID = new PID(this.config.thetaPIDConfig());
        timer = new Timer();
    }

    public void init(Supplier<Pose2d> odometry) {
        this.odometry = odometry;
        xPID.setMeasurement(() -> odometry.get().getX());
        yPID.setMeasurement(() -> odometry.get().getY());
        thetaPID.setMeasurement(() -> odometry.get().getRotation().getRadians());
        timer.reset();
        timer.start();
        currentSection = path.get(0);
        currentSection.calculatePath();
    }

    public ChassisSpeeds calculate() {
        if (currentSection.endOfPath(odometry.get())) {
            section++;
            if (section < path.size()) {
                currentSection = path.get(section);
                currentSection.calculatePath();
                timer.reset();
                timer.start();
            }
        }

        double time = timer.get();
        ChassisSpeeds speeds = currentSection.getSpeedsAtTime(time);
        applyPIDCorrection(currentSection.getPoseAtTime(time), speeds);

        return speeds;
    }

    public void runCommand() {
        if (isCommand())
            currentSection.runCommand();
    }

    public boolean isFinished() {
        if (section < path.size())
            return false;
        return currentSection.endOfPath(odometry.get());
    }

    public boolean isCommand() {
        return currentSection.isCommand();
    }

    public Pose2d getPose(double time) {
        return currentSection.getPoseAtTime(time);
    }

    private void applyPIDCorrection(Pose2d desiredPose, ChassisSpeeds speeds) {
        speeds.vxMetersPerSecond += xPID.calculate(desiredPose.getX());
        speeds.vyMetersPerSecond += yPID.calculate(desiredPose.getY());
        speeds.omegaRadiansPerSecond += thetaPID.calculate(desiredPose.getRotation().getRadians());
    }

}
