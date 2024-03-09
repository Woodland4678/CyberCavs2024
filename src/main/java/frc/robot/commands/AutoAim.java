// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoAim extends Command {
   PIDController rController = new PIDController(0.12,0.1,0.01); //0.13 0.4 0.01
   //ProfiledPIDController profiledRController = new ProfiledPIDController(0.1, 0.001, 0.007, new Constraints(4, 10));
   SwerveSubsystem S_Swerve;
   Shooter S_Shooter;
   Intake S_Intake;
   double targetX = 0;
   double rSpeed = 0;
   double startingYaw = 0;
   int count = 0;
   int lostTargetCount = 0;
   double currentTargetPitch = 0;
   double degrees = 0;
   double currentShooterAngleTarget = Constants.ShooterConstants.shooterStartingAngle;
   CommandXboxController driverController;
   boolean hasTarget = false;
   boolean isDone = false;
   boolean isShooting = false;
   int isShootingCnt = 0;
   boolean isAuto = false;
   boolean hasShot = false;
   int hasShotCnt = 0;
  /** Creates a new AutoAim. */
  public AutoAim(SwerveSubsystem S_Swerve, Shooter S_Shooter, Intake S_Intake, CommandXboxController driverController, boolean isAuto) {
    this.driverController = driverController;
    this.S_Swerve = S_Swerve;
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    count = 0;
    this.isAuto = isAuto;
    rController.setIZone(4);
    addRequirements(S_Swerve, S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    isShootingCnt = 0;
    isShooting = false;
    hasTarget = false;
    currentTargetPitch = 0;
    lostTargetCount = 0;
    hasShot = false;
    hasShotCnt = 0;
    if (S_Swerve.hasAprilTagTarget()) {
      hasTarget = true;
      startingYaw = S_Swerve.getAprilTagX();
      rController.setSetpoint(S_Swerve.getHeading().getDegrees() - (startingYaw - Constants.Swerve.autoAimYawOffset));
     // profiledRController.setGoal(S_Swerve.getHeading().getDegrees() - (startingYaw - Constants.Swerve.autoAimYawOffset));
      currentTargetPitch = S_Swerve.getAprilTagY();
      currentShooterAngleTarget = S_Shooter.calculateShooterAngle(currentTargetPitch);
    }
    else {
      rController.setSetpoint(0); //was 180
      //profiledRController.setGoal(0);
    }
    S_Shooter.setShooterAngle(currentShooterAngleTarget);
    S_Shooter.setRightAndLeftRPM(-3400, -4400);
    rController.setTolerance(4);
    //profiledRController.setTolerance(2);  
   // rController.setSetpoint(3.9);
   // S_Shooter.setRightAndLeftRPM(Constants.ShooterConstants.subwooferShotRightRPM, Constants.ShooterConstants.subwooferShotLeftRPM);
    //S_Shooter.setShooterAngle(64);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Swerve.hasAprilTagTarget()) {
      lostTargetCount = 0;
      hasTarget = true;
      startingYaw = S_Swerve.getAprilTagX();
      rController.setSetpoint(S_Swerve.getHeading().getDegrees() - (startingYaw - Constants.Swerve.autoAimYawOffset));
     // profiledRController.setGoal(S_Swerve.getHeading().getDegrees() - (startingYaw - Constants.Swerve.autoAimYawOffset));
      currentTargetPitch = S_Swerve.getAprilTagY();
      currentShooterAngleTarget = S_Shooter.calculateShooterAngle(currentTargetPitch);
      S_Shooter.setShooterAngle(currentShooterAngleTarget);
      //S_Shooter.setShooterAngle(currentShooterAngleTarget);
    }
    else {
      lostTargetCount++;
      if (lostTargetCount > 25) {
        hasTarget = false;
        rController.setSetpoint(0);
       // profiledRController.setGoal(0);
        currentShooterAngleTarget = Constants.ShooterConstants.shooterStartingAngle;
        S_Shooter.setShooterAngle(currentShooterAngleTarget);
      }
    }
     degrees = S_Swerve.getHeading().getDegrees();
    // if (rController.getSetpoint() > 0 && degrees < 0) {
    //   degrees = 360 + degrees;
    // }
    // else if (rController.getSetpoint() < 0 && degrees > 0) {
    //   degrees = degrees - 360;
    // }
    SmartDashboard.putBoolean("Shoot r controller at setpoint", rController.atSetpoint());
    SmartDashboard.putNumber("Shoot r controller error", rController.getPositionError());
    SmartDashboard.putNumber("Shoot shooter angle error", Math.abs(currentShooterAngleTarget - S_Shooter.getAnglePosition()));
    // rSpeed = rController.calculate(S_Swerve.getAprilTagX());
    rSpeed = rController.calculate(degrees);
    S_Swerve.drive(new Translation2d(-Math.pow(driverController.getLeftY(), 3) * S_Swerve.getMaximumVelocity(),
                                          -Math.pow(driverController.getLeftX(), 3) * S_Swerve.getMaximumVelocity()),
                        rSpeed,
                        true);
    if (Math.abs(S_Shooter.getLeftVelocity() - (-4400)) < 200
     &&  Math.abs(S_Shooter.getRightVelocity() - (-3400)) < 200 
     && rController.atSetpoint()
     && S_Shooter.setShooterAngle(currentShooterAngleTarget) < 0.8
     && hasTarget) {
     count++;
     if (count > 3) {
      isShooting = true;
      //S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerShootSpeed);
        //S_Intake.setHorizontalPercentOutput(-Constants.IntakeConstants.horizontalRollerShootSpeed);
        S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
        S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
        
     }
    }
    else {
      count = 0;
      if (!isAuto) {
        S_Intake.stopIntakeMotors();
      }
    }
    if (isShooting) {
      if (Math.abs(S_Shooter.getLeftVelocity() - (S_Shooter.getLeftTargetVelocity())) > 400) {
        hasShot = true;
      }
      //SmartDashboard.putNumber("Is shooting cnt", isShootingCnt);
    }
    if (hasShot && isAuto) {
      hasShotCnt++;
      if (hasShotCnt > 10) {
        isDone = true;
      }
    }
   // rController.setPID(S_Swerve.getAutoAimP(), S_Swerve.getAutoAimI(), S_Swerve.getAutoAimD());
    //rController.setIZone(S_Swerve.getAutoAimIZ());
  //profiledRController.setPID(S_Swerve.getAutoAimP(), S_Swerve.getAutoAimI(), S_Swerve.getAutoAimD());
  //profiledRController.setIZone(S_Swerve.getAutoAimIZ());
   
    //ChassisSpeeds robotSpeed = new ChassisSpeeds(0,0.5,rSpeed);
    //S_Swerve.setChassisSpeeds(robotSpeed);
    SmartDashboard.putNumber("rspeed", rSpeed);
    SmartDashboard.putNumber("Auto Aim Error", Math.abs(S_Swerve.getAprilTagX() - rController.getSetpoint()));
    SmartDashboard.putNumber("LeftRPmError", Math.abs(S_Shooter.getLeftVelocity() - (S_Shooter.getLeftTargetVelocity())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!isAuto) {
      S_Shooter.stopShooterMotor();
    }
    S_Intake.stopIntakeMotors();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

