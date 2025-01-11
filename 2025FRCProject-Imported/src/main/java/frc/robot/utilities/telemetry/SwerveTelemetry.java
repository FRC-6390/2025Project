package frc.robot.utilities.telemetry;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import frc.robot.utilities.swerve.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.utilities.controller.Debouncer;
import frc.robot.utilities.controlloop.PID;

public class SwerveTelemetry {
    SwerveModule fl;
    SwerveModule fr; 
    SwerveModule bl; 
    SwerveModule br; 
    PID driftCorrection; 
    ShuffleboardTab tab;
    SwerveDriveOdometry odom;
    Field2d field;

    public SwerveTelemetry(SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br, PID driftCorrection, SwerveDriveOdometry odom, Field2d field, ShuffleboardTab tab)
    {
        this.fl = fl;
        this.fr = fr; 
        this.bl = bl; 
        this.br = br; 
        this.odom = odom;
        this.driftCorrection = driftCorrection; 
        this.tab = tab; 
    }

    public void updateShuffleboard()
    {
        tab.add("Heading", odom.getPoseMeters().getRotation().getDegrees());
        tab.add("Front Left Encoder Position", fl.getPostion());
        tab.add("Front Right Encoder Position", fr.getPostion());
        tab.add("Back Right Encoder Position", br.getPostion());
        tab.add("Back Left Encoder Position", bl.getPostion());
        tab.add("Odometry Pose", odom.getPoseMeters());
        tab.add("Field Widget", field);
        tab.add("Drift Correction PID",driftCorrection);
        tab.add("Module Rotation PID", SwerveModule.rotationPidController);
    }



}
