// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANBusID{
        public static final int kLeftMotor1 = 7;
        public static final int kRightMotor1 = 12;
        public static final int kLeftMotor2 = 4;
        public static final int kRightMotor2 = 11;
        public static final int kLeftFlywheel = 2;
        public static final int kRightFlywheel = 1;
        public static final int dintakemotor = 10;
        public static final int conveyor1 = 20;
        public static final int conveyor2 = 21;
        public static final int dexotor = 9;
    }
    public static final class Ports {
        public static final int LeftDriveEncoder1 = 0;
        public static final int LeftDriveEncoder2 = 1;
        public static final int RightDriveEncoder1 = 2;
        public static final int RightDriveEncoder2 = 3;
    }

    public static final class ConveyorC{
        public static final double speed=0.3;
    }

    public static final class MovePID{    
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double f = 0.0;
        public static final double MaxAlignSpeed = 24; //Inches per second
        public static final double AlignTolerance = 12; //Inches
  }
    public static final class TurnPID{
        public static final double kTurnP = 2e-2;
        public static final double kTurnI = 1e-3; 
        public static final double kTurnD = 0.0001; 
        public static final double kTurnF = 0.4;
        public static final double kTurnToleranceDeg = 2.0;
        public static final double kTurnRateToleranceDegPerS = 100.0;

    }
    public static final class DriveC {    
        public static final double reducedMaxSpeed = 0.5;
        public static final double MaxSpeed = 1.0;
        public static final boolean SquareInputs = true;
        public static final int kEncoderCPR = 20;
        public static final double kWheelCircInches = 18.5;
        public static final double distancePerPulse = (kWheelCircInches) / (double) kEncoderCPR/10.71;//Gear Ratio = 1:10.71
    }

    public static final class Flywheel {
        public static final int waitforshootersecs = 10;
        public static final double feedmotorspeed = 0.5;
        public static final double shootermotorspeed = 2000;
    
    }
    public static final class firstIntake {
        public static final double firstIntakespeed = 0.5;
    }
    public static final class buttonsLeftjoy {
        public static final int halfspeedButton = 5;
        public static final int toggleIntakeExtend = 6;
        public static final int moveButton = 7;
        public static final int turnleftButton = 3;
        public static final int turnrightButton = 4;
        public static final int toggleIntake = 2;
    }

    public static final class buttonsRightjoy {
        public static final int Button = 1;
    }
}
