package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MovePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A command that will turn the robot to the specified angle.
 */

public class Move extends PIDCommand {
  /**
   * Moves robot to the distance measured on the LIDAR
   *
   * @param targetDistanceInches The Target distance for the LIDAR
   * @param drive                The drive subsystem to use
   * @param lidar                The LIDAR subsystem to use
   * @param kMoveP               Proportional feedback parameter
   * @param kMoveI               Integral feedback parameter
   * @param kMoveD               Derivative feedback parameter
   */

  public Move(double targetDistanceInches, DriveTrain drive) {
    super(new PIDController(MovePID.P, MovePID.I, MovePID.D),
        drive::getLeftDistance, targetDistanceInches, output -> drive.ArcadeDrive(-output, 0), drive);

    drive.resetEncoders();

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(MovePID.AlignTolerance, MovePID.MaxAlignSpeed);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
