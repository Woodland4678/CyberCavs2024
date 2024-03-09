// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDown extends Command {
  /** Creates a new ClimberDown. */
  Climber S_Climber;
  boolean isDone = false;
  public ClimberDown(Climber S_Climber) {
    this.S_Climber = S_Climber;
    addRequirements(S_Climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!S_Climber.isClimberDown()) {
      S_Climber.setClimberSpeed(0.2);
    }
    else {
      S_Climber.stopClimber();
      isDone = true;
      S_Climber.setPosition(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
