// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoGrabNote extends Command {
  SwerveSubsystem S_Swerve;
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
  /** Creates a new AutoGrabNote. */
  public AutoGrabNote(SwerveSubsystem S_Swerve) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    rController.setTolerance(Constants.Swerve.autoGrabNote_R_Tolerance);
    grabState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(grabState) {
      case 0:
        if (S_Swerve.limelightHasTarget() == 1) {
            degrees = S_Swerve.getHeading().getDegrees();
            // if (degrees < 0) {
            //   degrees += 360;
            // }
            //var boundingBoxXY = s_Swerve.getBoundingBoxX();
            rSpeed = rController.calculate(degrees);
            xSpeed = xController.calculate(S_Swerve.getLimelightX());
            double yMeasurement = S_Swerve.getLimelightY();
                  
            ySpeed = yController.calculate(yMeasurement);                     
            driveSpeed = new ChassisSpeeds(ySpeed, -xSpeed, rSpeed);
            
            SmartDashboard.putNumber("rSpeed", rSpeed);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            S_Swerve.setChassisSpeeds(driveSpeed);
            
            // if ((minLidarReading > 31 && minLidarReading < 43.5 - (ySpeed * 10)) && xController.atSetpoint() && currentArmError < 1.75 && currentTarget == Constants.ArmConstants.grabCubePosition) {             
              
            //   grabState++;
            //   s_Swerve.stop();                         
            // }                          
        }
        else {
          S_Swerve.stop();
        }
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
    return false;
  }
}
