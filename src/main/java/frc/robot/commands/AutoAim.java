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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class AutoAim extends Command {
   PIDController rController = new PIDController(0.18,0.00,0);
   SwerveSubsystem S_Swerve;
   double targetX = 0;
   double rSpeed = 0;
   double startingYaw = 0;
   CommandXboxController driverController;
  /** Creates a new AutoAim. */
  public AutoAim(SwerveSubsystem S_Swerve, CommandXboxController driverController) {
    this.driverController = driverController;
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingYaw = S_Swerve.getAprilTagX();
    //rController.setSetpoint(S_Swerve.getHeading().getDegrees() - startingYaw);
    rController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     rSpeed = rController.calculate(S_Swerve.getAprilTagX());
    S_Swerve.drive(new Translation2d(Math.pow(-driverController.getLeftY(), 3) * 2,
                                          Math.pow(-driverController.getLeftX(), 3) * 2),
                        rSpeed,
                        true);
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
    S_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

