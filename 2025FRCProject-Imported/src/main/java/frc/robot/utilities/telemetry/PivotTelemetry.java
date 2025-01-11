package frc.robot.utilities.telemetry;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utilities.swerve.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.utilities.controller.Debouncer;
import frc.robot.utilities.controlloop.PID;

public class PivotTelemetry {
    TalonFX motor;
    PID pid; 
    ShuffleboardTab tab;
    Mechanism2d arm;

    public PivotTelemetry(TalonFX motor, PID pid, ShuffleboardTab tab, Mechanism2d arm)
    {
        this.arm = arm;
        this.pid = pid;
        this.motor = motor;
        this.tab = tab; 
    }

    public void updateShuffleboard()
    {
        tab.add("Pivot Visual",arm);
        tab.add("PID Controller", pid);
        tab.add("Motor Position", motor.getRotorPosition().refresh().getValueAsDouble());
        tab.add("Motor Velocity", motor.getRotorVelocity().refresh().getValueAsDouble());
    }



}
