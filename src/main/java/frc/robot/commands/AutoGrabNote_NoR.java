// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Sides;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LEDStrip;
import frc.robot.LEDStrip.LEDModes;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoGrabNote_NoR extends Command {
  SwerveSubsystem S_Swerve;
  Intake S_Intake;
  PIDController xController = new PIDController(Constants.Swerve.autoGrabNote_X_P, Constants.Swerve.autoGrabNote_X_I, Constants.Swerve.autoGrabNote_X_D);
  PIDController yController = new PIDController(Constants.Swerve.autoGrabNote_Y_P, Constants.Swerve.autoGrabNote_Y_I, Constants.Swerve.autoGrabNote_Y_D);
  PIDController rController = new PIDController(Constants.Swerve.autoGrabNote_R_P, Constants.Swerve.autoGrabNote_R_I, Constants.Swerve.autoGrabNote_R_D);
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  int grabState = 0;
  double degrees = 0;
  boolean isDone = false;
  int waitCnt = 0;
  ChassisSpeeds driveSpeed;
  boolean isAuto = false;
  int noteCloseCount = 0;
  int runCnt = 0;
  int hasSeenNoteCount = 0;
 // private LEDStrip ledStrip;
  /** Creates a new AutoGrabNote. */
  public AutoGrabNote_NoR(SwerveSubsystem S_Swerve, Intake S_Intake, boolean isAuto) {
    this.S_Swerve = S_Swerve;
    this.S_Intake = S_Intake;
    this.isAuto = isAuto;
    //ledStrip = LEDStrip.getInstance();
    addRequirements(S_Swerve, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeenNoteCount = 0;
    // runCnt++;
    // if (isAuto && runCnt == 1) {
    //   yController.setP(0.08);
    // }
    // else {
    //   yController.setP(Constants.Swerve.autoGrabNote_Y_P);
    // }
    noteCloseCount = 0;
    rController.reset();
    xController.reset();
    yController.reset();
    S_Swerve.setHeadlights(true);
    isDone = false;
    waitCnt = 0;
    rSpeed = 0;
    xSpeed = 0;
    ySpeed = 0;
    S_Swerve.setLimelightPipeline(0); //was 0
    S_Swerve.setLimelightLED(false);
    xController.setSetpoint(Constants.Swerve.autoGrabNote_X_Target);
    xController.setTolerance(Constants.Swerve.autoGrabNote_X_Tolerance);
    yController.setSetpoint(Constants.Swerve.autoGrabNote_Y_Target);
    yController.setTolerance(Constants.Swerve.autoGrabNote_Y_Tolerance);
    rController.setSetpoint(S_Swerve.getHeading().getDegrees());
    //rController.setSetpoint(0);
    rController.setTolerance(Constants.Swerve.autoGrabNote_R_Tolerance);
    grabState = 0;
    S_Intake.setHorizontalPercentOutput(Constants.IntakeConstants.horizontalRollerIntakeSpeed);
    S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerIntakeSpeed);
    S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.indexerIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("Note Close Count", noteCloseCount);
   // rController.setPID(S_Swerve.getAutoAimP(), S_Swerve.getAutoAimI(), S_Swerve.getAutoAimD());
    //rController.setIZone(S_Swerve.getAutoAimIZ());
    switch(grabState) {
      case 0:
        if (S_Swerve.limelightHasTarget() == 1) {
          hasSeenNoteCount++;
          if (S_Swerve.getLimelightY() < 0) {
            noteCloseCount++;
          }
            // degrees = S_Swerve.getHeading().getDegrees();
            // if (rController.getSetpoint() > 0 && degrees < 0) {
            //   degrees = 360 + degrees;
            // }
            // else if (rController.getSetpoint() < 0 && degrees > 0) {
            //   degrees = degrees - 360;
            // }
           
            rSpeed = rController.calculate(degrees);
            xSpeed = xController.calculate(S_Swerve.getLimelightX());
            double yMeasurement = S_Swerve.getLimelightY();
                  
            ySpeed = yController.calculate(yMeasurement);                     
            driveSpeed = new ChassisSpeeds(-ySpeed, xSpeed, rSpeed); //no x speed for now
            
            SmartDashboard.putNumber("rSpeed", rSpeed);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            S_Swerve.setChassisSpeeds(driveSpeed);
            
            // if ((minLidarReading > 31 && minLidarReading < 43.5 - (ySpeed * 10)) && xController.atSetpoint() && currentArmError < 1.75 && currentTarget == Constants.ArmConstants.grabCubePosition) {             
              
            //   grabState++;
            //   s_Swerve.stop();                         
            // }  
            if (S_Intake.getPieceAquired() || S_Intake.isNoteOnRamp()) {
              grabState++;
              //LEDStrip.LEDModes = LEDMode.
            }     
            if (yMeasurement < -10) {
              waitCnt++;
            }           
            else {
              waitCnt = 0;
            }        
            if (waitCnt > 15) {
              grabState ++;
            }
        }
        else {
          if (noteCloseCount > 2) {
            driveSpeed = new ChassisSpeeds(1, 0, 0);
            S_Swerve.setChassisSpeeds(driveSpeed);
          }
          else if (isAuto && hasSeenNoteCount < 5) {
            driveSpeed = new ChassisSpeeds(0, 0, 1.75);
            rController.setSetpoint(S_Swerve.getHeading().getDegrees());
            S_Swerve.setChassisSpeeds(driveSpeed);
          } else {
            S_Swerve.stop();
          }
          if (S_Intake.isNoteOnRamp()) {
            grabState++;
            //LEDStrip.setLEDMode(LEDModes.BLINKGREEN);
          }
        }
      break;
      case 1:
        S_Swerve.stop();
        isDone = true;
        LEDStrip.getInstance().setLEDMode(LEDModes.BLINKGREEN);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Swerve.setHeadlights(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
