// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



public class NormalShoot extends Command {
  /** Creates a new Shoot. */
  int excecute = 0;
  Shooter S_Shooter;
  SwerveSubsystem S_Swerve;
  Intake S_Intake;
  public NormalShoot(Shooter S_Shooter, SwerveSubsystem S_Swerve, Intake S_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.S_Shooter = S_Shooter;
    this.S_Swerve = S_Swerve;
    this.S_Intake = S_Intake;
    addRequirements(S_Shooter, S_Swerve, S_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    S_Shooter.setShooterPIDFToSpinUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = 20; // Some velocity
    switch (excecute) {
      case 0:
        double angle = 60; //Some angle
        S_Swerve.AutoAim(); 
        S_Shooter.setRightAndLeftRPM(velocity, -velocity);
        if (S_Shooter.setShooterAngle(angle) < 3) {
          if (Math.abs(S_Shooter.getRightVelocity() - velocity) > 50 && Math.abs(S_Shooter.getLeftVelocity() + velocity) > 50) {
            if (S_Swerve.AutoAim() < 5) {
              excecute ++;
            }          
          }
        }
        break;
      case 1:
        S_Shooter.setShooterPIDFToMaintain();
        excecute++;
        break;
      case 2:
        S_Intake.setIndexMotorPercentOutput(0.7);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.setIndexMotorPercentOutput(0);
    S_Shooter.stopShooterMotor();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
