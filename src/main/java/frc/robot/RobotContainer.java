// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drivetrain = new DriveTrain();
  private final FirstIntake m_intake = new FirstIntake();
  public final Flywheel m_flywheel = new Flywheel();
  private final Joystick m_leftJoy = new Joystick(0);
  private final Joystick m_rightJoy = new Joystick(1);
  private final Climber m_climber = new Climber();
  private final Conveyor m_conveyor = new Conveyor();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    SendableRegistry.setName(m_drivetrain, "DriveTrain", "DriveTrain");
    SendableRegistry.setName(new Turn(90, m_drivetrain), "Turn Right command");
    SendableRegistry.setName(new Turn(-90, m_drivetrain), "Turn Left command");

    // SendableRegistry.setName(new InstantCommand(m_climber::climberMoter1on),
    // "Turn Climber1 on");
    // SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor1),
    // "Reverse Climber1");
    SendableRegistry.setName(new Conv1(m_conveyor), "Conveyor1On");
    SendableRegistry.setName(new Conv2(m_conveyor), "Conveyor2On");
    SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor1Reversed), "Reverse Conveyor1");
    SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor2Reversed), "Reverse Conveyor2");
    SendableRegistry.setName(m_flywheel, "Flywheel");
    SendableRegistry.setName(m_conveyor, "Conveyor");
    SendableRegistry.setName(m_intake, "Intake");
    // SendableRegistry.setName(m_climber, "Climber");

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain, () -> m_leftJoy.getY(), () -> m_rightJoy.getY()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Lower max speed
    // new JoystickButton(m_leftJoy, buttonsLeftjoy.halfspeedButton).whenPressed(()
    // -> m_drivetrain.toggleMaxSpeed());
    // new JoystickButton(m_leftJoy, buttonsRightJoy.intakeoffButton).whenPressed(()
    // -> m_intake.);
    new JoystickButton(m_leftJoy, buttonsRightjoy.pickupButton).whenPressed(new Pickup(m_intake, m_conveyor));
    new JoystickButton(m_leftJoy, buttonsRightjoy.shootButton).whenPressed(new Shoot(m_flywheel, m_conveyor));
    new JoystickButton(m_leftJoy, buttonsRightjoy.pickupOffButton).whenPressed(new PickupOff(m_intake, m_conveyor));
    /*
     * new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleIntake).whenPressed(()->
     * m_intake.toggleFirstIntake());
     * new JoystickButton(m_rightJoy, buttonsRightjoy.moveButton).whenPressed( new
     * Move(100.0,m_drivetrain).withTimeout(5));
     * new JoystickButton(m_rightJoy,
     * buttonsRightjoy.turnrightButton).whenPressed(new Turn(90,
     * m_drivetrain).withTimeout(5));
     * new JoystickButton(m_rightJoy,
     * buttonsRightjoy.turnleftButton).whenPressed(new Turn(-90,
     * m_drivetrain).withTimeout(5));
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleIntakeExtend).whenPressed(()->
     * m_intake.toggleExtension());
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleconveyor1).whenPressed(()->
     * m_conveyor.toggleconveyor1());
     * new JoystickButton(m_leftJoy,
     * buttonsLeftjoy.toggleconveyor2).whenPressed(()->
     * m_conveyor.toggleconveyor2());
     */

    // driveBaseTab.add("Climber Up", new
    // InstantCommand(m_climber::climberMoter1on));
    // driveBaseTab.add("Climber Off", new
    // InstantCommand(m_climber::climberMoter1off));
    // driveBaseTab.add("Climber Down", new
    // InstantCommand(m_climber::reverse_Motor1));
    // driveBaseTab.add("Flywheel On", new
    // InstantCommand(m_flywheel::shooteron));
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("DriveTrain");
    driveBaseTab.add("Tank Drive", m_drivetrain);
    driveBaseTab.add("Reset Gyro", new InstantCommand(m_drivetrain::resetHeading));
    driveBaseTab.add("Reset DT Encoders", new InstantCommand(m_drivetrain::resetEncoders));

    ShuffleboardTab IntakeTab = Shuffleboard.getTab("Intake");
    IntakeTab.add("IntakeRetractOFf", new InstantCommand(m_intake::stopRetract));
    IntakeTab.add("Intake Down", new InstantCommand(m_intake::intakeDown));
    IntakeTab.add("Intake Up", new InstantCommand(m_intake::intakeUp));
    IntakeTab.add("Toggle extend", new InstantCommand(m_intake::toggleExtension));
    IntakeTab.add("Intake On", new InstantCommand(m_intake::startFirstIntake));
    IntakeTab.add("Intake Off", new InstantCommand(m_intake::stopFirstIntake));
    IntakeTab.add("Intake Toggle", new InstantCommand(m_intake::toggleFirstIntake));
    ShuffleboardTab ShooterTab = Shuffleboard.getTab("Shooter");
    ShooterTab.add("Easy Shooter", new InstantCommand(m_flywheel::shooterGo));
    ShooterTab.add("Flywheel Off", new InstantCommand(m_flywheel::shooteroff));

    // driveBaseTab.add("Intake Reverse", new
    // InstantCommand(m_intake::reverseFirstIntake));
    
    driveBaseTab.add("Intake Reverse", new InstantCommand(m_intake::reverseFirstIntake));
    // driveBaseTab.add("Conveyor1 OnTill", new Conv1(m_conveyor));
    // driveBaseTab.add("Conveyor2 OnTill", new Conv2(m_conveyor));
    driveBaseTab.add("Climber Up", new InstantCommand(m_climber::climber_up));
    driveBaseTab.add("Climber Down", new InstantCommand(m_climber::climber_down));
    driveBaseTab.add("Climber Off", new InstantCommand(m_climber::climberMoter1off));

    // driveBaseTab.add("Conveyor1 Reverse", new
    // InstantCommand(m_conveyor::conveyor1Reversed));
    // driveBaseTab.add("Conveyor2 Reverse", new
    // InstantCommand(m_conveyor::conveyor2Reversed));
    driveBaseTab.add("Conveyor2 Reverse", new InstantCommand(m_conveyor::conveyor2Reversed));
    driveBaseTab.add("Conveyor1 Toggle", new InstantCommand(m_conveyor::toggleconveyor1));
    driveBaseTab.add("Conveyor2 Toggle", new InstantCommand(m_conveyor::toggleconveyor2));
    driveBaseTab.add("Pickup", new Pickup(m_intake, m_conveyor));
    driveBaseTab.add("Pickup Off", new PickupOff(m_intake, m_conveyor));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(m_flywheel::shooterGo),
        new Pause(2.0),
        new InstantCommand(m_conveyor::conveyor2ON),
        // new Shoot(m_flywheel, m_conveyor),
        new InstantCommand(m_drivetrain::toggleMaxSpeed),
        new Pause(2.0),
        new Move(-60, m_drivetrain),
        new InstantCommand(m_drivetrain::toggleMaxSpeed),
        new InstantCommand(m_flywheel::shooteroff),
        new InstantCommand(m_conveyor::conveyor2OFF),
        new InstantCommand(m_intake::intakeDown));
  }

}
