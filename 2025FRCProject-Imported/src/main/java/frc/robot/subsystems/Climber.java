package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.Button;
import frc.robot.utilities.sensors.IRBBSensor;

 
public class Climber extends SubsystemBase {
 
  public static TalonFX climbMotorLeft;
  public static TalonFX climbMotorRight;
  public static DigitalInput leftLimitSwitch;
  public static DigitalInput rightLimitSwitch;

 
  public Climber()
  {
    
    //leftLimitSwitch = new DigitalInput(5);
    //rightLimitSwitch = new DigitalInput(7);
    motorBrake();
  }
 
  static
  {
 
  climbMotorLeft = new TalonFX(10, Constants.DRIVETRAIN.CANBUS);
  climbMotorRight = new TalonFX(16, Constants.DRIVETRAIN.CANBUS);
  TalonFXConfiguration config = new TalonFXConfiguration();
  CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
  curr.SupplyCurrentLimitEnable = true;
  curr.SupplyCurrentLimit = 30;
  
  
  config.withCurrentLimits(curr);

  climbMotorLeft.getConfigurator().apply(config);
  climbMotorRight.getConfigurator().apply(config);

  climbMotorLeft.setNeutralMode(NeutralModeValue.Brake);
  climbMotorRight.setNeutralMode(NeutralModeValue.Brake);

  
  }
 

  public void setRollers(double speed){
  climbMotorLeft.set(speed);
  climbMotorRight.set(-speed);

 
  }
 
public static boolean leftLimit(){

  return !leftLimitSwitch.get();
}

public static boolean rightLimit(){

  return !rightLimitSwitch.get();
}
 
  public static StatusSignal<Current> getRollerCurrent()
  {
    return climbMotorLeft.getSupplyCurrent();
  }
 

  public void motorBrake(){
    climbMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    climbMotorRight.setNeutralMode(NeutralModeValue.Brake);
  }
 
  public void motorCoast(){
    climbMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    climbMotorRight.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic()
  {
    
    
    // if (leftLimit()){
    //   climbMotorLeft.set(0);
    // }
    // if (rightLimit()){
    //   climbMotorRight.set(0);
    // }

  }
}