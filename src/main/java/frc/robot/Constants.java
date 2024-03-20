// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.ActionMap;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (108 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(24)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ShooterConstants {
    public static final int shooterRightMotorCanID = 9;
    public static final int shooterLeftMotorCanID = 10;
    public static final int shooterAngleMotorCanID = 19;
    //public static final int shooterAngleEncoderAbsoluteID = 4;
    public static final double shooter_SpinUp_P = 0.0003;
    public static final double shooter_SpinUp_I = 0.0;
    public static final double shooter_SpinUp_D = 0;
    public static final double shooter_SpinUp_FF = 0.00016;
    public static final double shooter_Maintain_P = 0.10;
    public static final double shooter_Maintain_I = 0;
    public static final double shooter_Maintain_D = 0;
    public static final double angleP = 0.04;
    public static final double angleI = 0.00001;
    public static final double angleD = 0;
    public static final double angleFF = 0;
    //public static final double angleAbsoluteOffset = 0;
    public static final double shooterStartingAngle = 64;
    public static final double angleGearRatio = 116.6666666666667;
    public static final double angleAngleConversionFactor = 360.0 / angleGearRatio;
    //public static final double restAngle = 85; // TODO Just a guess, change when robot is built
    public static final int subwooferShotRPMTolerance = 100;
    public static final int subwooferShotAngleTolerance = 5;
    public static final double subwooferShotAngle = 64;
    public static final double subwooferShotLeftRPM = -4000;
    public static final double subwooferShotRightRPM = -2500;

    public static final double minShooterAngle = 64.0;
    public static final double maxShooterAngle = 94.0;

    public static final double AmpSideQuickShootTwoAngle = 75;
  }
  public static class Swerve {
    public static final int headlightsRelayChannel = 3;
    public static final double autoGrabNote_X_P = 0.05;
    public static final double autoGrabNote_X_I = 0;
    public static final double autoGrabNote_X_D = 0;
    public static final double autoGrabNote_X_Tolerance = 1;
    public static final double autoGrabNote_X_Target = 0;

    public static final double autoGrabNote_Y_P = 0.18;
    public static final double autoGrabNote_Y_I = 0;
    public static final double autoGrabNote_Y_D = 0;
    public static final double autoGrabNote_Y_Tolerance = 1;
    public static final double autoGrabNote_Y_Target = -12;

    public static final double autoGrabNote_R_P = 0.11;
    public static final double autoGrabNote_R_I = 0;
    public static final double autoGrabNote_R_D = 0;
    public static final double autoGrabNote_R_Tolerance = 3;

    public static final double rotateRobot_P = 0.06;
    public static final double rotateRobot_I = 0;
    public static final double rotateRobot_D = 0;

    public static final double autoAimYawOffset = 0.5; //was 3.9
  }
  public static class ArmConstants {
    public static final int armShoulderMotorCanID = 11;
    public static final int armElbowMotorCanID = 12;
    public static final int armWristMotorCanID = 13;
    public static final int armRollerMotorCanID = 14;
    public static final int shoulderEncoderAbsoluteID = 4;
    public static final int elbowEncoderAbsoluteID = 5;
    public static final double shoudlerP = 0.03;
    public static final double shoulderI = 0;
    public static final double shoulderD = 0.1;
    public static final double elbowP = 0.015;
    public static final double elbowI = 0;
    public static final double elbowD = 1;
    public static final double wristP = 0.006;
    public static final double wristI = 0.00005;
    public static final double wristD = 0;
    public static final double wristIz = 5;
    public static final int wristMagSensorMaxValue = 950;
    public static final double shoulderAbsoluteOffset = 139.5; //arm resting against the limelight holder, at 40 degrees from horizontal
    public static final double elbowAbsoluteOffset = 301.4; 
    public static final double shoulderGearRatio = (25.0/1.0) * (60.0/24.0);
    public static final double elbowGearRatio = 25/1;
    public static final double wristGearRatio = 84.0/29.0 * 76.0/21.0;
    public static final double shoulderAngleConversionFactor = 360 / shoulderGearRatio;
    public static final double elbowAngleConversionFactor = 360 / elbowGearRatio;
    public static final double wristAngleConversionFactor = 360.0 / wristGearRatio;
    public static final double shoulderLength = 19.07;
    public static final double elbowLength = 19.40;
    public static final double intakeRollerPosition = 15.4; // Some Number, TODO 18 for trap

    public static final ArmPosition restPosition = new ArmPosition(4.7, 4.2, 0, false);
    public static final ArmPosition startPosition = new ArmPosition(4.7, 4.2, 0, false); //same as rest
    public static final ArmPosition AmpIntermediatePos1 = new ArmPosition(132, -154, 0, true);
    public static final ArmPosition AmpIntermediatePos2 = new ArmPosition(5.55, 17.63, 0, false);
    public static final ArmPosition trapIntermediatePos2 = new ArmPosition(1, 30.63, 0, false);
    public static final ArmPosition ampScoring = new ArmPosition(7, 31, -35, false); // Approximate x and y values
    public static final ArmPosition trapScoring = new ArmPosition(70, 0, -105.76, true); //Approximate x and y values
    public static final ArmPosition testPos = new ArmPosition(11, 29, 0, false);
    public static final ArmPosition testPos1 = new ArmPosition(8, 16, 0, false);
    public static final int hasNoteSensor = 7;
    public static final int wristHomeSensorChannel = 0;
    public static final double armIntakeSpeed = 0.9;
  }
  public static class IntakeConstants {
    public static final int IntakeVerticalRollerCanID = 15;
    public static final int IntakeHorizontalRollerCanID = 16;
    public static final int indexerCanID = 17;
    public static final int indexerMotorCanID = 20;
    public static final int pieceAquireChannel = 6;
    public static final int diverterUpChannel = 1;
    public static final int noteOnRampSensor = 9;
    public static final double indexRollerShootSpeed = -0.9;
    public static final double rampRollerShooterSpeed = 0.8;
    public static final double horizontalRollerShootSpeed = 0.7;
    public static final double verticalRollerShootSpeed = 0.7;

    public static final double horizontalRollerIntakeSpeed = -1;
    public static final double verticalRollerIntakeSpeed = 1;
    public static final double indexerIntakeSpeed = 1;

    public static final double horizontalRollerOuttakeSpeed = 0.9;
    public static final double verticalRollerOuttakeSpeed = -0.9;
    public static final double indexerOuttakeSpeed = -0.9;
  }
  public static class ClimberConstants {
    public static final int ClimberMainMotorCanID = 18;
    public static final int climberDownChannel = 8;
    public static final double climberP = 0.1;
    public static final double climberI = 0;
    public static final double climberD = 0;

    public static final double grabHighPosition = 10; //TODO get actual value
    public static final double grabLowPosition = 5; //TODO get actual value
    public static final double scorePosition = 2; //TODO get actual value

    public static final double minClimberPosition = 10; //TODO get actual value
    public static final double maxClimberHeight = -80;

  }
  public static class ArmPosition{
    public double xTarget = 10; //default "homeish" position
    public double yTarget = 15; //default "homeish" position
    public double wristPitchTarget = 0;
    public boolean isAngleTarget = false;
    public ArmPosition(double xTarget, double yTarget, double wristPitchTarget, boolean isAngleTarget) {
      this.xTarget = xTarget;
      this.yTarget = yTarget;
      this.wristPitchTarget = wristPitchTarget;
      this.isAngleTarget = isAngleTarget;
    }
  }
}
