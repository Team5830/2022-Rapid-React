package frc.robot.commands;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.TurnPID;
import frc.robot.subsystems.DriveTrain;

/**
 * A command that will turn the robot to the specified angle.
 */
public class Turn extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public Turn(double targetAngleDegrees, DriveTrain drive) {
    super(new PIDController(TurnPID.kTurnP, TurnPID.kTurnI, TurnPID.kTurnD), 
    drive::getHeading, targetAngleDegrees, output -> drive.ArcadeDrive(0, output), drive);
        
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(TurnPID.kTurnToleranceDeg, TurnPID.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}




//public class Rotate90 extends CommandBase {
  /**
   * Creates a new Rotate90.
   */
  /*
   private final Rotate m_drivetrain;
  private double startangle;
  public Rotate90(Rotate drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startangle = m_drivetrain.getMeasurement();
    m_drivetrain.setTarget(startangle+90);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.enable();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(m_drivetrain.finished());
  }
}
*/
