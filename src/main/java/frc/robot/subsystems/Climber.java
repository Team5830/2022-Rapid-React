
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax climberMotorup;
  RelativeEncoder m_encoderup;
  double fsoftlimit = Constants.ClimberC.climberforwardlimit;
  double rsoftlimit = Constants.ClimberC.climberreverselimit;
  boolean isclimberMotorupon = false;
  public boolean isclimberMotorupreversed = false;

  public Climber() {
    try {
      climberMotorup = new CANSparkMax(Constants.CANBusID.climberMotorup, MotorType.kBrushless);
      climberMotorup.restoreFactoryDefaults();
      m_encoderup = climberMotorup.getEncoder();
      climberMotorup.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      climberMotorup.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      climberMotorup.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
          (float) Constants.ClimberC.climberforwardlimit);
      climberMotorup.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
          (float) Constants.ClimberC.climberreverselimit);
      SmartDashboard.putNumber("Forward Soft Limit", fsoftlimit);
      SmartDashboard.putNumber("Reverse Soft Limit", rsoftlimit);
      m_encoderup.setPosition(0.0);
      // climberMotorup.setInverted(isInverted);
      // m_encoderup = climberMotorup=getClimberState();
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
    ShuffleboardTab ClimberControl = Shuffleboard.getTab("Climber");
    ClimberControl.add("Encoder", m_encoderup.getPosition());
    ClimberControl.add("Forward Soft Limit", fsoftlimit);
    ClimberControl.add("Reverse Soft Limit", rsoftlimit);
    ClimberControl.add("Reverse dir", isclimberMotorupreversed);
  }

  @Override
  public void periodic() {
    // climberMotorup.setInverted(true);
    climberMotorup.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotorup.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    fsoftlimit = SmartDashboard.getNumber("Forward Soft Limit", Constants.ClimberC.climberforwardlimit);
    rsoftlimit = SmartDashboard.getNumber("Reverse Soft Limit", Constants.ClimberC.climberreverselimit);
    climberMotorup.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) fsoftlimit);
    climberMotorup.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) rsoftlimit);
    SmartDashboard.putNumber("Climber Encoder", m_encoderup.getPosition());
  }

  public void climber_up() {
    if (isclimberMotorupreversed) {
      climberMotorup.set(-Constants.ClimberC.climberSpeed);
    } else {
      climberMotorup.set(Constants.ClimberC.climberSpeed);
    }
  }

  public void climber_down() {
    if (isclimberMotorupreversed) {
      climberMotorup.set(Constants.ClimberC.climberSpeed);
    } else {
      climberMotorup.set(-Constants.ClimberC.climberSpeed);
    }
  }

  public void reverse_Motor1() {
    if (isclimberMotorupreversed) {
      isclimberMotorupreversed = false;
    } else {
      isclimberMotorupreversed = true;
    }
  }

  public void climberMoter1on() {
    if (isclimberMotorupreversed) {
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

  public void toggleMotor1() {
    if (isclimberMotorupon) {
      isclimberMotorupon = false;
    } else {
      isclimberMotorupon = true;
    }
  }
}
