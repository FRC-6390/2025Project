// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.vision.LimeLight;

public class TelemetryManager extends SubsystemBase {
  Drivetrain6390 drivetrain;
  LimeLight limelightDriver;
  LimeLight limelightTag;
  Arm arm;
  Intake intake;
  Shooter shooter;
  ShuffleboardTab swerve = Shuffleboard.getTab("Swerve");
  ShuffleboardTab limelights = Shuffleboard.getTab("Limelights");
  ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  ShuffleboardTab sequencing = Shuffleboard.getTab("Sequencing");

  public TelemetryManager(Drivetrain6390 drivetrain, LimeLight limelightDriver, LimeLight limelightTag, Arm arm, Intake intake, Shooter shooter) 
  {
    this.drivetrain = drivetrain;
    this.limelightDriver = limelightDriver;
    this.limelightTag = limelightTag;
    this.arm = arm;
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void periodic() 
  {
    //DRIVETRAIN
    // swerve.add("Vision Pose", drivetrain.getVisionPose());
    // swerve.add("Drivetrain", drivetrain.getHeading());
    
    // //LIMELIGHTS
    // limelights.add("LimelightDriver Sees Note?", limelightDriver.hasValidTarget());
    // limelights.add("LimelightTag Sees Tag?", limelightTag.hasValidTarget());

    // //ARM
    // armTab.add("Arm Position", arm.getPostionAsPercent());
    // armTab.add("Arm PID", arm.PID);
    // armTab.add("Arm", arm);

    // //INTAKE
    // intakeTab.add("Game Piece", intake.hasNote());
    // intakeTab.add("Intake",intake);

    // //SHOOTER
    // shooterTab.add("Shooter Speed", shooter.getRotorVelocity());
    // shooterTab.add("Shooter", shooter);

    // //COMMAND SCHEDULER
    // sequencing.add("Command Scheduler",CommandScheduler.getInstance());
  }
}
