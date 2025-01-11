// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FixNote extends Command {

  public Shooter shooter;
  public Intake intake;
  public boolean reversed;
  
  public FixNote(Shooter shooter, Intake intake, boolean reversed) {
 
    this.shooter = shooter;
    this.intake = intake;
    this.reversed = reversed;
    addRequirements(shooter, intake);
  }
  /** Creates a new StopShooter. */
  public FixNote() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    intake.setReversed(reversed);
    intake.setOverride(true);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(reversed)
    {
      shooter.setRollers(0.3);
    }
    else
    {
      shooter.setRollers(-0.3);
    }
    //shooter.setPID(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.setOverride(false);
    intake.setReversed(false);
    shooter.setRollers(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
