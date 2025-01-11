// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightConfig;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.LedMode;
import frc.robot.utilities.vision.LimeLight.PoseEstimateType;

public class AprilTagAlign extends Command {
  public LimeLight limelight; 
  public Drivetrain6390 drivetrain;
  public DebouncedController cont; 
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController xController = new PIDController(1.225, 0, 0);
  public double thetaSpeed = 0;

  public AprilTagAlign(String limeLight, Drivetrain6390 drivetrain, DebouncedController cont) {
    this.drivetrain = drivetrain; this.limelight = new LimeLight(new LimelightConfig(limeLight, 0, 0)); this.cont = cont;
  }

  @Override
  public void initialize() 
  {
  }

  @Override
  public void execute() 
  {
    if(DriverStation.isTeleop())
    {
    if(limelight.hasValidTarget()){
      if(Math.abs(limelight.getTargetHorizontalOffset()) > 35)
      {
        drivetrain.drive(
        new ChassisSpeeds(
          cont.leftY.getAsDouble(), 
          0, 
          // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
          controller.calculate(limelight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
          );
      }
      else
      {
        drivetrain.drive(
          new ChassisSpeeds(
            cont.leftY.getAsDouble(), 
            -xController.calculate(limelight.getTargetHorizontalOffset()), 
            // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
            controller.calculate(limelight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
            );
      }

       }
      }
    else if(DriverStation.isAutonomous())
    {
    if(limelight.hasValidTarget()){
      if(Math.abs(limelight.getTargetHorizontalOffset()) > 35)
      {
        drivetrain.YDisable(true);
        drivetrain.feedbackDrive(
        new ChassisSpeeds(
          0, 
          0, 
          // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
          controller.calculate(limelight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
          );
      }
      else
      {
        drivetrain.feedbackDrive(
          new ChassisSpeeds(
            cont.leftY.getAsDouble(), 
            -xController.calculate(limelight.getTargetHorizontalOffset()), 
            // controller.calculate(LimelightHelpers.getBotPose_TargetSpace(limelight)[4])
            controller.calculate(limelight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getRotation().getDegrees()))
            );
      }

       }
       else
       {
        drivetrain.YDisable(false);
        drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
       }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.feedbackDrive(new ChassisSpeeds(0,0,0));
    drivetrain.YDisable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
