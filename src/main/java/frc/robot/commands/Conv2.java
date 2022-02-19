package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Conv2 extends CommandBase {
    Conveyor spriva2;
    public Conv2(Conveyor SprivA2) {
        spriva2 =SprivA2;
        addRequirements(spriva2);
    }
  
    @Override
    public void execute() {
      spriva2.DigiConvey2();
    }
  }
  