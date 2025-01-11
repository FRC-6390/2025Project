// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.vision.LimeLight;
import frc.robot.utilities.vision.LimelightConfig;
import frc.robot.utilities.vision.LimelightHelpers;
import frc.robot.utilities.vision.LimeLight.LedMode;

public class LinedUpSignal extends Command {
  // public LimeLight limelight; 
  public boolean precisionMode;
  public LimeLight limeLight;
  public double tolerance;
  
  public LinedUpSignal(String limeLight, boolean precisionMode, double tolerance) 
  { 
    this.limeLight = new LimeLight(new LimelightConfig(limeLight, 0, 0));
    
    this.precisionMode = precisionMode;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(precisionMode == false)
    {
    // if(LimelightHelpers.getTV(limelight))
    if(limeLight.hasValidTarget())
    {
      // LimelightHelpers.setLEDMode_ForceOn(limelight);
      limeLight.setLedMode(LedMode.ON);
    }
    else
    {
      // LimelightHelpers.setLEDMode_ForceOff(limelight);
      limeLight.setLedMode(LedMode.OFF);
    }
  }
  else
  {
    if(limeLight.hasValidTarget() && Math.abs(limeLight.getTargetHorizontalOffset()) < tolerance)
    {
      // LimelightHelpers.setLEDMode_ForceOn(limelight);
      limeLight.setLedMode(LedMode.ON);
    }
    else
    {
      limeLight.setLedMode(LedMode.OFF);
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
