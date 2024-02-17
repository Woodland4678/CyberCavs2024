// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class PassNoteToArm extends Command {
  int pause;
  int state;
  boolean isDone = false;
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
    state = 0;
    pause = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
    case 0:
      S_Arm.MoveArm(Constants.ArmConstants.restPosition);
      S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerShootSpeed);
      S_Intake.setIndexMotorPercentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
        
      if (S_Intake.isDiverterDown() && S_Arm.MoveArm(Constants.ArmConstants.restPosition) < 4) {
        state ++;
      }
        
      break;

    case 1:
      pause ++;
      S_Intake.stopIntakeMotors();
      if (pause > 5) {
        state ++;
      }
      break;

    case 2:
      S_Intake.setVerticalPercentOutput(- (Constants.IntakeConstants.verticalRollerShootSpeed));
      S_Intake.setIndexMotorPercentOutput(- (Constants.IntakeConstants.indexRollerShootSpeed));
      S_Arm.setRollerOutputPercent(Constants.ArmConstants.armIntakeSpeed);
       
      if (S_Arm.getHasNote()) {
        isDone = true;
       }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();
    S_Arm.stopArmRollers();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}