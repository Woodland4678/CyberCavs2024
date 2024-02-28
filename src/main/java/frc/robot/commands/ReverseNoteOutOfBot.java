// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ReverseNoteOutOfBot extends Command {
  Intake S_Intake;
  /** Creates a new ReverseNoteOutOfBot. */
  public ReverseNoteOutOfBot(Intake S_Intake) {
    this.S_Intake = S_Intake;
    addRequirements(S_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Intake.isDiverterDown() == false){
      S_Intake.setVerticalPercentOutput(- (Constants.IntakeConstants.verticalRollerShootSpeed));
      S_Intake.setRampRollerMotorPercentOutput(- (Constants.IntakeConstants.indexRollerShootSpeed));
    }
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
