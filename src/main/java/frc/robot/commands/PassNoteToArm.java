// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class PassNoteToArm extends Command {
  boolean isDone = false;
  int state = 0;
  Arm S_Arm;
  Intake S_Intake;
  /** Creates a new PassNoteToArm. */
  public PassNoteToArm(Arm S_Arm, Intake S_Intake) {
    this.S_Arm = S_Arm;
    this.S_Intake = S_Intake;
    addRequirements(S_Arm, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
    case 0:
      S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerShootSpeed);
      S_Intake.setIndexMotorPercentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
        
      if (S_Intake.isDiverterDown() == true) {
        state ++;
      }
        
      break;
         
    case 1:
      S_Intake.setVerticalPercentOutput(- (Constants.IntakeConstants.verticalRollerShootSpeed));
      S_Intake.setIndexMotorPercentOutput(- (Constants.IntakeConstants.indexRollerShootSpeed));
       
      if () {
        isDone = true;
       }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
