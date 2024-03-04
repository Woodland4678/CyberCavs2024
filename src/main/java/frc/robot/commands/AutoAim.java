// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoAim extends Command {
   PIDController rController = new PIDController(0.13,0.00,0);
   SwerveSubsystem S_Swerve;
   Shooter S_Shooter;
   Intake S_Intake;
   double targetX = 0;
   double rSpeed = 0;
   double startingYaw = 0;
   double count = 0;
   CommandXboxController driverController;
  /** Creates a new AutoAim. */
  public AutoAim(SwerveSubsystem S_Swerve, Shooter S_Shooter, Intake S_Intake, CommandXboxController driverController) {
    this.driverController = driverController;
    this.S_Swerve = S_Swerve;
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    count = 0;
    addRequirements(S_Swerve, S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingYaw = S_Swerve.getAprilTagX();
    //rController.setSetpoint(S_Swerve.getHeading().getDegrees() - startingYaw);
    rController.setSetpoint(5.2);
    S_Shooter.setRightAndLeftRPM(Constants.ShooterConstants.subwooferShotRightRPM, Constants.ShooterConstants.subwooferShotLeftRPM);
    S_Shooter.setShooterAngle(64);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     rSpeed = rController.calculate(S_Swerve.getAprilTagX());
    S_Swerve.drive(new Translation2d(Math.pow(driverController.getLeftY(), 1) * 1,
                                          Math.pow(driverController.getLeftX(), 1) * 1),
                        rSpeed,
                        true);
    if (Math.abs(S_Shooter.getLeftVelocity() - Constants.ShooterConstants.subwooferShotLeftRPM) < 100
     &&  Math.abs(S_Shooter.getRightVelocity() - Constants.ShooterConstants.subwooferShotRightRPM) < 100 
     && Math.abs(S_Swerve.getAprilTagX() - rController.getSetpoint()) < 4
     && S_Shooter.setShooterAngle(64) < 1) {
     count++;
     if (count > 3) {
      S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerShootSpeed);
        //S_Intake.setHorizontalPercentOutput(-Constants.IntakeConstants.horizontalRollerShootSpeed);
        S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
        S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
     }
    }
    else {
      count = 0;
      S_Intake.stopIntakeMotors();
    }
    //rController.setPID(S_Swerve.getAutoAimP(), S_Swerve.getAutoAimI(), S_Swerve.getAutoAimD());
    //rController.setIZone(S_Swerve.getAutoAimIZ());
   
    //ChassisSpeeds robotSpeed = new ChassisSpeeds(0,0.5,rSpeed);
    //S_Swerve.setChassisSpeeds(robotSpeed);
    SmartDashboard.putNumber("rspeed", rSpeed);
    SmartDashboard.putNumber("Auto Aim Error", Math.abs(S_Swerve.getAprilTagX() - rController.getSetpoint()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Shooter.stopShooterMotor();
    S_Intake.stopIntakeMotors();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

