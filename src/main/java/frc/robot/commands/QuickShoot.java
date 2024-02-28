// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class QuickShoot extends Command {
  Shooter S_Shooter;
  Intake S_Intake;
  int count = 0;
  boolean isDone = false;
  /** Creates a new QuickShoot. */
  public QuickShoot(Shooter S_Shooter, Intake S_Intake) {
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    addRequirements(S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    S_Intake.setRampRollerMotorPercentOutput(0.8);
    if (count >= 10) {
      isDone = true;
    }
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.setRampRollerMotorPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
