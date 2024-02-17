// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
  Climber S_Climber;
  double climberPosition;
  /** Creates a new MoveClimber. */
  public MoveClimber(Climber S_Climber, double climberPosition) {
    this.S_Climber = S_Climber;
    this.climberPosition = climberPosition;
    addRequirements(S_Climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    S_Climber.disengageLock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO move to climberPosition

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
