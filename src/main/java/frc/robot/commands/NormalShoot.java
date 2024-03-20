// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



public class NormalShoot extends Command {
  /** Creates a new Shoot. */
  int state = 0;
  Shooter S_Shooter;
  SwerveSubsystem S_Swerve;
  Intake S_Intake;
  double velocity;
  double shooterAngle;
  double robotAngle;
  double rSpeed;
 
  ChassisSpeeds driveSpeed;
  PIDController rController = new PIDController(Constants.Swerve.rotateRobot_P, Constants.Swerve.rotateRobot_I, Constants.Swerve.rotateRobot_D);
  public NormalShoot(Shooter S_Shooter, SwerveSubsystem S_Swerve, Intake S_Intake, double velocity, double shooterAngle, double robotAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.S_Shooter = S_Shooter;
    this.S_Swerve = S_Swerve;
    this.S_Intake = S_Intake;
    this.velocity = velocity;
    this.shooterAngle = shooterAngle;
    this.robotAngle = robotAngle;
    addRequirements(S_Shooter, S_Swerve, S_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     Optional<Alliance> ally = DriverStation.getAlliance();
    state = 0;
    //S_Shooter.setShooterPIDFToSpinUp();
    rController.setIZone(5);
    rController.setSetpoint(robotAngle);
    rController.setTolerance(4);
    if (ally.get() == Alliance.Red) {
        robotAngle = robotAngle * -1;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case 0:
      rSpeed = rController.calculate(S_Swerve.getHeading().getDegrees());
      driveSpeed = new ChassisSpeeds(0, 0, rSpeed);
      S_Swerve.setChassisSpeeds(driveSpeed);
      
      S_Shooter.setRightAndLeftRPM(-4800, -4000);
        if (S_Shooter.setShooterAngle(shooterAngle) < 1) {
          if (Math.abs(S_Shooter.getRightVelocity() - S_Shooter.getRightTargetVelocity()) < 100 && Math.abs(S_Shooter.getLeftVelocity() - S_Shooter.getLeftTargetVelocity()) < 100) {
            if (rController.atSetpoint()) {  
              state ++;
            }
          }
        }
        break;
      case 1:
      S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
      S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();
    S_Shooter.stopShooterMotor();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
