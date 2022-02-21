package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Conv1 extends CommandBase {
    Conveyor spriva;
    public Conv1(Conveyor SprivA) {
        spriva =SprivA;
        addRequirements(spriva);
    }
    @Override
    public void initialize() {
      spriva.ballaway1 = false;
      spriva.ballsensed1 = false;
    }
    @Override
    public void execute() {
      spriva.DigiConvey1();
    }
    @Override
    public void end(boolean interrupted) {
      spriva.conveyor1OFF();
    }
    @Override
    public boolean isFinished() {
      return spriva.ballaway1;
    }
  }
  