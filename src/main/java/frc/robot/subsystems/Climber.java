
package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;   

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climberMotorup;
  CANSparkMax m_encoderup; 
  boolean isclimberMotorupon = false;
  boolean isclimberMotorupreversed = false;

  public Climber() {
    try {
      climberMotorup = new CANSparkMax(Constants.CANBusID.climberMotorup, MotorType.kBrushless);
      climberMotorup.restoreFactoryDefaults();
      //m_encoderup = climberMotorup=getClimberState();
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    
  }
  
  public void climber_up(){
    climberMotorup.set(Constants.ClimberC.climberSpeed);
  }
  
  public void climber_down(){
    climberMotorup.set(-Constants.ClimberC.climberSpeed);
  }

  public void reverse_Motor1(){
    if (isclimberMotorupreversed){ 
      isclimberMotorupreversed = false;
    } else{
      isclimberMotorupreversed = true;
    }
    climberMoter1on();
  }

  public void climberMoter1on() {
    if (isclimberMotorupreversed){
        climberMotorup.set(-Constants.ClimberC.climberSpeed);
    } else {
        climberMotorup.set(Constants.ClimberC.climberSpeed);
    }
    isclimberMotorupon = true;
  }


  public void climberMoter1off() {
    climberMotorup.set(0);
    isclimberMotorupon = false;
  }

  public boolean readyToClimb() {
      // Should we check the robot position here?
    return true;
  }

    public boolean getClimberState() {
        return isclimberMotorupon;
    }

  public void toggleMotor1(){
    if(isclimberMotorupon) {
        isclimberMotorupon = false;
    } else {
        isclimberMotorupon = true;
    }
  }
}




