package frc.robot.commands;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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

    drive.resetHeading();

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
