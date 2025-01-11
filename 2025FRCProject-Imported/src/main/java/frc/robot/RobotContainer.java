// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightConfig;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.commands.auto.AprilTagAlign;
import frc.robot.commands.auto.SeekMode;
import frc.robot.commands.auto.TurnCommand;

public class RobotContainer {
  
  // public static Arm arm;
  public static LimeLight limelight = new LimeLight(new LimelightConfig("limelight", 0, 0));
  public static LimeLight limeLight2 = new LimeLight(new LimelightConfig("limelight-driver", 0, 0));
  public static Drivetrain6390 driveTrain = new Drivetrain6390(limelight);
  // public static Intake intake = new Intake();
  
  public static DebouncedController controller = new DebouncedController(0);
  private DebouncedJoystick joystick = new DebouncedJoystick(1);

 

  public RobotContainer() {
    // arm = new Arm(joystick);
    driveTrain.init();
    // NamedCommands.registerCommand("IntakeRollers", new IntakeRollers2(intake, -0.6, true));
    
    
   
    // NamedCommands.registerCommand("PivotMoveHigh", new ArmTest(arm, -0.211));
    NamedCommands.registerCommand("Seek", new SeekMode("limelight-driver", driveTrain));
    NamedCommands.registerCommand("Align", new AprilTagAlign("limelight-tag", driveTrain, controller));

    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX, controller));
    // intake.setDefaultCommand(new IntakeRollers2(intake, -0.4, true));
    configureBindings();

  }

  private void configureBindings() 
  {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.a.toggleOnTrue(new SeekMode("limelight-driver", driveTrain));
    controller.x.toggleOnTrue(new AprilTagAlign("limelight-tag",driveTrain, controller));
  }


  public Command getAutonomousCommand()
  {
  driveTrain.resetHeading();
  return new PathPlannerAuto("New Auto");
  }

}