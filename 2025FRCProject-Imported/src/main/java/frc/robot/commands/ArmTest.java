// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


/*THIS IS THE DEFAULT COMMAND. IN ROBOTCONTAINER, THIS COMMAND IS SET AS A DEFUALT COMMAND AND WILL ALWAYS RUN. SO, INSTEAD OF CALLING 
 * THE PID LOOP IN PERIODIC HERE, WE CREATE A VARIABLE IN ROBOTCONTAINER CALLED POS AND CONSTANTLY FEED THAT INTO THE DEFAULT COMMAND.
 * THEN WE UPDATE THE POS VARIABLE IN ROBOT CONTAINER BY CALLING THE SETARM COMMAND. THIS SHOULD WORK I THINK.
 */


public class ArmTest extends Command {
  double pos;

  public Arm test;
  public boolean isDone;

  /** Creates a new ArmTest. */
  public ArmTest(Arm test, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pos = pos;
    this.test = test;

    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //test.setPosition(pos);
    test.setPosition(pos);
    if(test.atPosition()||test.override())
    {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    test.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
