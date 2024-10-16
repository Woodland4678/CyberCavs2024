// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.LEDStrip;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FieldPass extends Command {
  SwerveSubsystem S_Swerve;
  Shooter S_Shooter;
  Intake S_Intake;
  CommandXboxController driverController;
  Optional<Alliance> ally;
  PIDController rController;
  /** Creates a new FieldPass. */
  public FieldPass(SwerveSubsystem S_Swerve, Shooter S_Shooter, Intake S_Intake, CommandXboxController driverController) {
    this.S_Swerve = S_Swerve;
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    this.driverController = driverController;
    rController = new PIDController(0.1,0.2,0.01); //0.13 0.4 0.01
    rController.setTolerance(1);
    rController.setIZone(2);
    addRequirements(S_Swerve, S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red) {
      rController.setSetpoint(35); //TODO figure out best angle
    }
    else {
      rController.setSetpoint(-35);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    S_Shooter.setRightAndLeftRPM(-2750, -3100);
    
    if ((S_Shooter.setShooterAngle(Constants.ShooterConstants.fieldPassAngle) < 0.9)  && rController.atSetpoint() && (Math.abs(S_Shooter.getLeftVelocity() - (S_Shooter.getLeftTargetVelocity())) < 500 &&  Math.abs(S_Shooter.getRightVelocity() - (S_Shooter.getRightTargetVelocity())) < 500)) {
      S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
      S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
    }

    S_Swerve.drive(new Translation2d(-Math.pow(driverController.getLeftY(), 3) * S_Swerve.getMaximumVelocity(),
                                          -Math.pow(driverController.getLeftX(), 3) * S_Swerve.getMaximumVelocity()),
                        rController.calculate(S_Swerve.getHeading().getDegrees()), //rController.calculate(S_Swerve.getHeading().getDegrees()
                        true);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
    S_Shooter.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
