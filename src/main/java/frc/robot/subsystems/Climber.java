
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;   

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climberMotor1;
  RelativeEncoder m_encoder1; 
  boolean isclimberMotor1on = false;
  boolean isclimberMotor1reversed = false;

  public Climber() {
    try {
      climberMotor1 = new CANSparkMax(Constants.CANBusID.climberMotor1, MotorType.kBrushless);
      climberMotor1.restoreFactoryDefaults();
      m_encoder1 = climberMotor1.getEncoder();
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    
  }
/*
  public boolean getClimberState1() {
    return isclimberon;
  }
*/
  public void reverse_Motor1(){
    if (isclimberMotor1reversed){ 
      isclimberMotor1reversed = false;
    } else{
      isclimberMotor1reversed = true;
    }
    climberMoter1on();
  }

  public void climberMoter1on() {
    if (isclimberMotor1reversed){
        climberMotor1.set(-Constants.ClimberC.climberSpeed);
    } else {
        climberMotor1.set(Constants.ClimberC.climberSpeed);
    }
    isclimberMotor1on = true;
  }

  public void climberMoter1off() {
    climberMotor1.set(0);
    isclimberMotor1on = false;
  }

  public boolean readyToClimb() {
      // Should we check the robot position here?
    return true;
  }

    public boolean getClimberState() {
        return isclimberMotor1on;
    }

  public void toggleMotor1(){
    if(isclimberMotor1on) {
        isclimberMotor1on = false;
    } else {
        isclimberMotor1on = true;
    }
  }
}




