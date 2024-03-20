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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateToStage extends Command {
  /** Creates a new RotateToStage. */
  SwerveSubsystem S_Swerve;
  double rotateTarget = 60;
  double rSpeed = 0;
  CommandXboxController driverController;
  PIDController rController = new PIDController(Constants.Swerve.rotateRobot_P, Constants.Swerve.rotateRobot_I, Constants.Swerve.rotateRobot_D);
  public RotateToStage(SwerveSubsystem S_Swerve, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.S_Swerve = S_Swerve;
    this.driverController = driverController;
    addRequirements(S_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.setIZone(5);
    rSpeed = 0;
    // Optional<Alliance> ally = DriverStation.getAlliance();
    // if (ally.get() == Alliance.Red) {
    //   rotateTarget = -90;
    // }
    // else {
    //   rotateTarget = 90;
    // }
    rController.setSetpoint(rotateTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rSpeed = rController.calculate(S_Swerve.getHeading().getDegrees());
    S_Swerve.drive(new Translation2d(-Math.pow(driverController.getLeftY(), 3) * S_Swerve.getMaximumVelocity(),
                                          -Math.pow(driverController.getLeftX(), 3) * S_Swerve.getMaximumVelocity()), rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
