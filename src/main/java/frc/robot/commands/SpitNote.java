// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SpitNote extends Command {
  Shooter S_Shooter;
  Intake S_Intake;
  /** Creates a new SpitNote. */
  public SpitNote(Shooter S_Shooter, Intake S_Intake) {
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    addRequirements(S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    S_Shooter.setShooterAngle(92);
    S_Shooter.setRightAndLeftRPM(-2000, -2000); //Experimented with -1400, -1400 and it worked well
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Shooter.setShooterAngle(91) < 2 && Math.abs(S_Shooter.getLeftVelocity() - S_Shooter.getLeftTargetVelocity()) < 200) {
      S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
      S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Shooter.stopShooterMotor();
    S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
    S_Intake.stopIntakeMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
