package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import com.fasterxml.jackson.databind.module.SimpleAbstractTypeResolver;
// import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Shooter extends SubsystemBase {
 

  public static TalonFX feedingRollers;
  public static TalonFX leftShooterMotor;
  public static TalonFX rightShooterMotor;
  public static double shooterSpeed;
  public static VelocityVoltage vel;
  public static Slot0Configs configs;
  public double idleSpeed = -0.2;
  public static boolean isIdle = false;
 
  

  public static boolean isIdle() {
    return isIdle;
  }

  public void setIdle(boolean isIdle) {
    this.isIdle = isIdle;
  }

  public double setpoint = 0;
  public boolean atSetpoint;
 
  public Shooter()
  {
   

  }
 
  static
  {
  leftShooterMotor = new TalonFX(Constants.SHOOTER.LEFT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  rightShooterMotor = new TalonFX(Constants.SHOOTER.RIGHT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  // rightShooterMotor.setInverted(true);
  isIdle = true;
}
  
  public void setRollers(double speed){
    shooterSpeed = speed;
  }

  public double getRotorVelocity()
  {
    return leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble();
  }
  public boolean atSetpoint(double targetSpeed)
  {
    return Math.abs(leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble()) >= Math.abs(targetSpeed); 
  }

  public void stopShooter()
  {
    shooterSpeed = 0;
    // System.out.println("//------------------SHOOTER IS STOPPED ------------------///");
  }

 
  public static StatusSignal<Current> getRollerCurrent()
  {
    return feedingRollers.getSupplyCurrent();
  }
  public void update()
  {
    if(shooterSpeed == 0 && isIdle)
    {
      rightShooterMotor.set(-idleSpeed);
      leftShooterMotor.set(idleSpeed);
    }
    else
    {
      rightShooterMotor.set(-shooterSpeed);
      leftShooterMotor.set(shooterSpeed);
    }
  }
 
  @Override
  public void periodic(){
    // update();
  
    // SmartDashboard.putNumber("Shooter Velocity", getRotorVelocity());
  }
}