// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public final Flywheel m_flywheel = new Flywheel();
  private final Joystick m_leftJoy = new Joystick(0);
  private final Joystick m_rightJoy = new Joystick(1);
  private final Climber m_climber = new Climber();
  private final Conveyor m_conveyor = new Conveyor();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    SendableRegistry.setName(new Move(24,m_drivetrain), "DriveTrain", "Move 2'"); //for testing
    SendableRegistry.setName(new Turn(90, m_drivetrain), "Turn Right command"); //for testing
    SendableRegistry.setName(new Turn(-90, m_drivetrain), "Turn Left command"); //for testing
    SendableRegistry.setName(new InstantCommand(m_climber::climberMoter1on),"Climber", "Climber1 On");
    SendableRegistry.setName(new InstantCommand(m_climber::climberMoter2on),"Climber", "Climber2 On");
    SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor1), "Climber","Reverse Climber1");
    SendableRegistry.setName(new InstantCommand(m_climber::reverse_Motor2), "Climber","Reverse Climber2");
    SendableRegistry.setName(new InstantCommand(m_flywheel::shooteron),"Flywheel", "On");
    SendableRegistry.setName(new InstantCommand(m_flywheel::shooteroff),"Flywheel", "Off");  
    SendableRegistry.setName(new InstantCommand(m_intake::extendIntake), "Intake","Extend Intake"); 
    SendableRegistry.setName(new InstantCommand(m_intake::retractIntake), "Intake","Retract Intake"); 
    SendableRegistry.setName(new InstantCommand(m_intake::toggleExtension), "Intake","Toggle extend");
    SendableRegistry.setName(new InstantCommand(m_intake::startFirstIntake), "Intake", "On");
    SendableRegistry.setName(new InstantCommand(m_intake::stopFirstIntake), "Intake", "Off"); 
    SendableRegistry.setName(new InstantCommand(m_intake::toggleFirstIntake), "Intake", "Toggle"); 
    SendableRegistry.setName(new InstantCommand(m_intake::reverseFirstIntake), "Intake", "Reverse");
    SendableRegistry.setName(new InstantCommand(m_drivetrain::resetHeading), "DriveTrain", "Reset Gyro"); 
    SendableRegistry.setName(new InstantCommand(m_drivetrain::resetEncoders), "DriveTrain", "Reset Encoders"); 
    SendableRegistry.setName(new InstantCommand(m_intake::reverseFirstIntake), "Intake", "Reverse"); 
    SendableRegistry.setName(new Conv1(m_conveyor), "Conveyor1On");
    SendableRegistry.setName(new Conv2(m_conveyor), "Conveyor2On");
    SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor1Reversed), "Reverse Conveyor1");
    SendableRegistry.setName(new InstantCommand(m_conveyor::conveyor2Reversed), "Reverse Conveyor2");
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
    /*
    new JoystickButton(m_rightJoy, buttonsRightjoy.moveButton).whenPressed( new Move(100.0,m_drivetrain).withTimeout(5));
    new JoystickButton(m_rightJoy, buttonsRightjoy.turnrightButton).whenPressed(new Turn(90, m_drivetrain).withTimeout(5));
    new JoystickButton(m_rightJoy, buttonsRightjoy.turnleftButton).whenPressed(new Turn(-90, m_drivetrain).withTimeout(5));
    */
    new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleIntakeExtend).whenPressed(()-> m_intake.toggleExtension());
    new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleconveyor1).whenPressed(()-> m_conveyor.toggleconveyor1());
    new JoystickButton(m_leftJoy, buttonsLeftjoy.toggleconveyor2).whenPressed(()-> m_conveyor.toggleconveyor2());
  
    SmartDashboard.putData("Move Command", new Move(100.0, m_drivetrain));
    SmartDashboard.putData("Turn Right Command", new Turn(90.0, m_drivetrain));
    SmartDashboard.putData("Turn Left Command", new Turn(-90.0, m_drivetrain));
    SmartDashboard.putData("Climber1 On", new InstantCommand(m_climber::climberMoter1on));
    SmartDashboard.putData("Climber2 On", new InstantCommand(m_climber::climberMoter2on));
    SmartDashboard.putData("Reverse Climber1", new InstantCommand(m_climber::reverse_Motor1));
    SmartDashboard.putData("Reverse Climber2", new InstantCommand(m_climber::reverse_Motor2));
    SmartDashboard.putData("Flywheel On",new InstantCommand(m_flywheel::shooteron));
    SmartDashboard.putData("Flywheel Off", new InstantCommand(m_flywheel::shooteroff));  
    SmartDashboard.putData("Extend Intake",new InstantCommand(m_intake::extendIntake)); 
    SmartDashboard.putData("Retract Intake", new InstantCommand(m_intake::retractIntake)); 
    SmartDashboard.putData("Toggle extend", new InstantCommand(m_intake::toggleExtension));
    SmartDashboard.putData("Intake On", new InstantCommand(m_intake::startFirstIntake));
    SmartDashboard.putData("Intake Off", new InstantCommand(m_intake::stopFirstIntake)); 
    SmartDashboard.putData("Intake Toggle", new InstantCommand(m_intake::toggleFirstIntake)); 
    SmartDashboard.putData("Intake Reverse", new InstantCommand(m_intake::reverseFirstIntake));
    SmartDashboard.putData("Reset Gyro", new InstantCommand(m_drivetrain::resetHeading)); 
    SmartDashboard.putData("Reset DT Encoders", new InstantCommand(m_drivetrain::resetEncoders)); 
    SmartDashboard.putData("Intake Reverse", new InstantCommand(m_intake::reverseFirstIntake)); 
    SmartDashboard.putData("Conveyor1On", new Conv1(m_conveyor));
    SmartDashboard.putData("Conveyor2On", new Conv2(m_conveyor));
    SmartDashboard.putData("Reverse Conveyor1", new InstantCommand(m_conveyor::conveyor1Reversed));
    SmartDashboard.putData("Reverse Conveyor2", new InstantCommand(m_conveyor::conveyor2Reversed));
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
                                                                                                                                                                