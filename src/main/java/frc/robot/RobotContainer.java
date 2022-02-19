// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_drivetrain = new DriveTrain();
  private final FirstIntake m_intake = new FirstIntake();
  //public final Flywheel m_flywheel = new Flywheel();
  private final Joystick m_leftJoy = new Joystick(0);
  private final Joystick m_rightJoy = new Joystick(1);
  private final Climber m_climber = new Climber();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    SendableRegistry.setName(m_drivetrain, "DriveTrain", "DriveTrain");
    SendableRegistry.setName(new Turn(90, m_drivetrain), "Turn Right command");
    SendableRegistry.setName(new Turn(-90, m_drivetrain), "Turn Left command");
    SendableRegistry.setName(new InstantCommand(m_climber::climberMoter1on), "Turn Climber1 on");
    SendableRegistry.setName(new InstantCommand(m_climber::climberMoter2on), "Turn Climber2 on");
    SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor1), "Reverse Climber1");
    SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor2), "Reverse Climber2");
    //SendableRegistry.setName(new InstantCommand(m_flywheel::setpoint?, "Flywheel", "Flywheel");    
    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain, () -> m_leftJoy.getY(), () -> m_rightJoy.getY()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Lower max speed
    new JoystickButton(m_leftJoy, buttonsLeftjoy.halfspeedButton).whenPressed(()-> m_drivetrain.toggleMaxSpeed());
    new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleIntake).whenPressed(()-> m_intake.toggleFirstIntake());
    //may be changed to toggle later
    new JoystickButton(m_leftJoy, buttonsLeftjoy.moveButton).whenPressed( new Move(100.0,m_drivetrain).withTimeout(5));
    new JoystickButton(m_leftJoy, buttonsLeftjoy.turnrightButton).whenPressed(new Turn(90, m_drivetrain).withTimeout(5));
    new JoystickButton(m_leftJoy, buttonsLeftjoy.turnleftButton).whenPressed(new Turn(-90, m_drivetrain).withTimeout(5));
    //new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleIntake).whenPressed(()-> m_intake.toggleFirstIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

}
                                                                                                                                                                