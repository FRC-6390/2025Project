// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.time.TimeDelay;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightConfig;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.LedMode;

public class IntakeRollers2 extends Command {
  /** Creates a new Intake. */
  public Intake intake;
  public double speed;
  public boolean useBeamBreak;
  public LimeLight limeLight = new LimeLight(new LimelightConfig("limelight-tag", 0, 0));
  public boolean isHomeSet;
  public double startTime;
  public boolean didInit = false;
  public IntakeRollers2(Intake intake, double speed, boolean useBeamBreak) {
    addRequirements(intake);
    this.intake = intake;
    this.speed = speed;
    this.useBeamBreak = useBeamBreak;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    intake.setRollersEnabled(true);
    intake.fullWidth(speed);
    intake.centerIntake(speed);
    intake.feed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double curtime = Timer.getFPGATimestamp();
    double rotations = 0;
    if(DriverStation.isAutonomous())
    {
      rotations = 80;
    }
    else
    {
      rotations = 50;
    }
    if(intake.hasNote())
    {
     
      if(!didInit)
      {
        limeLight.setLedMode(LedMode.ON);
        // LimelightHelpers.setLEDMode_ForceOn("limelight");
        startTime = Timer.getFPGATimestamp();
        didInit = true;
      }
      if((curtime - startTime) > 1)
      {
        limeLight.setLedMode(LedMode.OFF);
        // LimelightHelpers.setLEDMode_ForceOff("limelight");
      }
      if(!isHomeSet)
      {
        intake.setHome();
        isHomeSet = true;
      }
      if(intake.getRotations() < rotations)
      {
        intake.setRollersEnabled(false);
        intake.setReversed(false);
     }
    }
    else
    {
      didInit = false;
      intake.setRollersEnabled(true);
      isHomeSet = false;
      limeLight.setLedMode(LedMode.OFF);
      // LimelightHelpers.setLEDMode_ForceOff("limelight");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.setRollersEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
