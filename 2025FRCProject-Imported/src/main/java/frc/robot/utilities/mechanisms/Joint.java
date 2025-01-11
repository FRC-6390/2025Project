// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.mechanisms;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.controlloop.PIDConfig;

/** Add your docs here. */
public class Joint extends SubsystemBase
{
    public record JointMotor(TalonFX motor, boolean isInverted) 
    {

    }

    public JointMotor leftJointRecord;
    public JointMotor rightJointRecord;
    public TalonFX leftJointDrive;
    public TalonFX rightJointDrive;
    public double maxPosition;
    public PIDController controller;
    public CurrentLimitsConfigs currentLimit;
    public double setpoint;

    public Joint
    (JointMotor leftMotor, JointMotor rightMotor, double maxPos,PIDController pid, CurrentLimitsConfigs curr)
    {
        this.leftJointRecord = leftMotor;
        this.rightJointRecord = rightMotor;
        this.maxPosition = maxPos;
        this.controller = pid;
        this.currentLimit = curr;
        leftJointDrive.getConfigurator().apply(currentLimit);
        rightJointDrive.getConfigurator().apply(currentLimit);
        if(leftMotor.isInverted())
        {
            leftJointDrive.setInverted(true);
        }
        if(rightMotor.isInverted())
        {
            rightJointDrive.setInverted(true);
        }
    }

    //RETURNS POSITION IN PERCENT
    public double getPosition()
    {
        return leftJointDrive.getRotorPosition().refresh().getValueAsDouble() / maxPosition;
    }

    //SET POSITION IN PERCENT
    public void setPosition(double position)
    {
        leftJointDrive.setPosition(position * maxPosition);
    }

    //SET SPEED PERCENT OUTPUT
    public void setSpeed(double speed)
    {
        leftJointDrive.set(speed);
        rightJointDrive.set(speed);
    }

    public void setAngle(double angleInPercent)
    {
        setpoint = angleInPercent;
    }

    public void lock()
    {
        leftJointDrive.setNeutralMode(NeutralModeValue.Brake);
        rightJointDrive.setNeutralMode(NeutralModeValue.Brake);
    }
    public void unlock()
    {
        leftJointDrive.setNeutralMode(NeutralModeValue.Coast);
        rightJointDrive.setNeutralMode(NeutralModeValue.Coast);
    }

    public void periodic()
    {
        setSpeed(controller.calculate(getPosition(), setpoint * maxPosition));
    }
}
