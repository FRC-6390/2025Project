package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.commands.auto.TurnCommand;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;

public class Drive extends Command {

  //Creates a drivetrain subsystem
  private Drivetrain6390 driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, yInput, thetaInput;
  //These are limiters. The make sure the rate of change is never too abrupt and smooth out inputs from the joystick.
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
  public double angle;
  public PIDController thetaController;
  private DebouncedController controller;
  public double rot;

  double kP = 0.08;
  double kI = 0;
  double kD = 0;

  public Drive(Drivetrain6390 driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput, DebouncedController controller) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    this.controller = controller;
    xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    thetaLimiter = new SlewRateLimiter(12.5664d);
    thetaController = new PIDController(kP, kI, kD);
    thetaController.enableContinuousInput(-180, 180);
    
    //YOU MUST HAVE THIS - WONT WORK OTHERWISE
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.lockWheels();
  }

  @Override
  public void execute() {
  if(DriverStation.isTeleop()){
    //Take the inputs from the joystick, dampen them and then put it into variables
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    // double x = -controller.getRightX();
    // double y = -controller.getRightY();
    // SmartDashboard.putNumber("x", x);
    // SmartDashboard.putNumber("y", y);
    // if(Math.abs(x) < 0.04 || Math.abs(y) < 0.04)
    // {
    // rot = driveTrain.getRotation2d().getDegrees();
    // }
    // else
    // {
    // rot = Math.toDegrees(Math.atan2(x,y));
    // }
    // SmartDashboard.putNumber("rot", rot);
    
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

  
    //Store the individual speeds into a single class
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, -thetaSpeed);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, driveTrain.getRotation2d());
    // chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, driveTrain.getRotation2d());
     

    //Feed that into the drive train subsystem
    driveTrain.drive(chassisSpeeds);
    
    // double speed = thetaController.calculate(driveTrain.getHeading(), rot);

    // SmartDashboard.putNumber("Sped", speed);  
    // driveTrain.feedbackDrive(
    //     new ChassisSpeeds(0,0,
    //     speed)
    //   );
   

    
    // CommandScheduler.getInstance().schedule(new TurnCommand(driveTrain, angle));
    
  }

   
  }

  @Override
  public void end(boolean interrupted) {
    //Shut it off
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}