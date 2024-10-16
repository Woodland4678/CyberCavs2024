// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoAdjustShooterAngle extends Command {
  Shooter S_Shooter;
  Intake S_Intake;
  SwerveSubsystem S_Swerve;
  int lostTargetCount = 0;
  double shooterAngleTarget = Constants.ShooterConstants.shooterStartingAngle;
  Optional<Alliance> ally;
  
  /** Creates a new AutoAdjustShooterAngle. */
  public AutoAdjustShooterAngle(Shooter S_Shooter, Intake S_Intake, SwerveSubsystem S_Swerve) {
    this.S_Shooter =  S_Shooter;
    this.S_Intake = S_Intake;
    this.S_Swerve = S_Swerve;
    addRequirements(S_Shooter);
    ally = DriverStation.getAlliance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lostTargetCount = 0;
    shooterAngleTarget = Constants.ShooterConstants.shooterStartingAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Intake.isNoteOnRamp()) {
      if (S_Swerve.hasAprilTagTarget()) {
        lostTargetCount = 0;
        shooterAngleTarget = S_Shooter.calculateShooterAngle(S_Swerve.getAprilTagY(), ally);
        S_Shooter.setShooterAngle(shooterAngleTarget);
      }
      else {
        lostTargetCount++;
        if (lostTargetCount > 25 && !DriverStation.isAutonomous()) {
          S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
        }
      }
    }
    else if (!DriverStation.isAutonomous()){
      //S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
