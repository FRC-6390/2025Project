// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightConfig;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.LedMode;

public class SeekMode extends Command {
  public LimeLight limelight; 
  public Drivetrain6390 drivetrain; 
  public PIDController controller = new PIDController(0.05, 0, 0);
  public double oldRot = 0;
  public boolean isDone;
  public boolean hasRecorded;
  public SeekMode(String limeLight, Drivetrain6390 drivetrain) 
  {
    this.drivetrain = drivetrain; this.limelight = new LimeLight(new LimelightConfig(limeLight, 0,0));
  }

  @Override
  public void initialize() 
  {
isDone = false;
  }


  @Override
  public void execute() 
  { 
    
  
    ChassisSpeeds speeds= new ChassisSpeeds(-1, 0, 0);
    // double xspeed = -1;
    // double yspeed = -1;
    // double xspeedRelative = (xspeed * Math.cos(drivetrain.getHeading())) -  (yspeed * Math.sin(drivetrain.getHeading()));
    // double yspeedRelative = (xspeed * Math.sin(drivetrain.getHeading())) + (yspeed * Math.cos(drivetrain.getHeading()));
    
    if(DriverStation.isTeleop())
    {
    System.out.println("SEEKING");
    if(limelight.hasValidTarget())
      { 
        drivetrain.feedbackDrive(new ChassisSpeeds(-3, 0, controller.calculate(limelight.getTargetHorizontalOffset())));
      }
      else
      {
        drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
      }
    }
    else
    {
      if(limelight.hasValidTarget())
      { 
        drivetrain.disableDrive(true);
        drivetrain.feedbackDrive(new ChassisSpeeds(-1, 0, controller.calculate(limelight.getTargetHorizontalOffset())));
      }
      else
      {
        drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
        drivetrain.disableDrive(false);

      }
      // drivetrain.disableDrive(true);
      // System.out.println("SEEKING");
      // if(LimelightHelpers.getTV(limelight))
      // { 
      //   speeds.toRobotRelativeSpeeds(drivetrain.getRotation2d());
      //   PPHolonomicDriveController.overrideXFeedback(() -> {return speeds.vxMetersPerSecond;});
      //   PPHolonomicDriveController.overrideYFeedback(() -> {return speeds.vyMetersPerSecond;});
      //   PPHolonomicDriveController.overrideRotationFeedback(() -> {return controller.calculate(LimelightHelpers.getTX(limelight));});
      // }
      // else
      // {
      //   PPHolonomicDriveController.clearFeedbackOverrides();
      // } 
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.disableDrive(false);
    PPHolonomicDriveController.clearFeedbackOverrides();
    drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  
  }
}
