// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.sensors.Button;

public class Arm extends SubsystemBase {
  private TalonFX ArmMotorLeft;
  private TalonFX ArmMotorRight;
  public PID PID;
  public double setpoint = 0;
  public double convertedValue = 0;
  public Button coastButton;
  private DebouncedJoystick joystick;
  private StatusSignal<Angle> rotorPos;
  public double maxPos = Constants.ARM.ARM_MAX;
  public StatusSignal<Current> amperage;
  public boolean shouldCoast;
 
  public Arm(DebouncedJoystick joystick) 
  {    
    ArmMotorLeft = new TalonFX(Constants.ARM.ARM_MOTOR_LEFT, Constants.DRIVETRAIN.CANBUS);
    ArmMotorRight = new TalonFX(Constants.ARM.ARM_MOTOR_RIGHT, Constants.DRIVETRAIN.CANBUS);
    coastButton = new Button(new DigitalInput(1));
    this.joystick = joystick;
    joystick.one.onFalse(new InstantCommand(this::stopAll));

    PID = new PID(Constants.ARM.PID_config);
    rotorPos = ArmMotorLeft.getRotorPosition();
    amperage = ArmMotorLeft.getTorqueCurrent();
    PID.setMeasurement(()->rotorPos.getValueAsDouble());
    ArmMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    ArmMotorRight.setNeutralMode(NeutralModeValue.Brake);
    CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
    curr.SupplyCurrentLimitEnable = true;
    curr.SupplyCurrentLimit = 80;
    ArmMotorLeft.getConfigurator().apply(curr);
    ArmMotorRight.getConfigurator().apply(curr);
    motorBrake();
  }

  public void setSpeed(double speed)
  {
    ArmMotorLeft.set(-speed);
    ArmMotorRight.set(speed);
  }

  public void setHome(){
    ArmMotorLeft.setPosition(0);
  }

  public void stopAll(){
    ArmMotorLeft.set(0);
    ArmMotorRight.set(0);
  }

  public boolean checkThreshold(){
   return true;
  }
  public void setPosition(double pos)
  {
  pos = Math.min(Math.max(pos, -1), 0.075);
  setpoint = pos;
  }

  public boolean atPosition()
  {
   return Math.abs(Math.abs(maxPos * setpoint) - Math.abs(rotorPos.getValueAsDouble())) < 0.6; 
  }

public boolean override(){
  return joystick.one.getAsBoolean();
}
 public void setHalf(){
  setPosition(0.5);
 }

 public void motorBrake(){
  ArmMotorLeft.setNeutralMode(NeutralModeValue.Brake);
  ArmMotorRight.setNeutralMode(NeutralModeValue.Brake);
}

public double percentToRotations(double percent, double rangeInRotations)
{
  return percent * rangeInRotations;
}

public double getPostionAsPercent()
{
  return ArmMotorLeft.getPosition().refresh().getValueAsDouble() / Constants.ARM.ARM_MAX;
}

public void motorCoast()
{
  ArmMotorLeft.setNeutralMode(NeutralModeValue.Coast);
  ArmMotorRight.setNeutralMode(NeutralModeValue.Coast);
}
  @Override
  public void periodic() {

    if (joystick.one.getAsBoolean())
    {
      double input = joystick.leftY.getAsDouble();
      double newRange = (0 - (-1));
      double oldRange = (1-(-1));
      double joystickPos = (((input - (-1)) * newRange)/oldRange)+(-1);
      setPosition(joystickPos);
    }

  double speed = PID.calculate(-percentToRotations(setpoint, maxPos));
  setSpeed(-speed);
  rotorPos.refresh();
  amperage.refresh();
  SmartDashboard.putNumber("Setpoitn", setpoint);

    
if (DriverStation.isDisabled()){
  if (coastButton.isPressed()){
    motorCoast();
  }
else{
motorBrake();
}
}}
}
