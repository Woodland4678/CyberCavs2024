// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PassNoteToArm extends Command {
  int pause;
  int state;
  boolean isDone = false;
  Arm S_Arm;
  int count = 0;
  Intake S_Intake;
  Shooter S_Shooter;
  /** Creates a new PassNoteToArm. */
  public PassNoteToArm(Arm S_Arm, Intake S_Intake, Shooter S_Shooter) {
    this.S_Arm = S_Arm;
    this.S_Intake = S_Intake;
    this.S_Shooter = S_Shooter;
    addRequirements(S_Arm, S_Intake, S_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    pause = 0;
    count = 0;
    isDone = false;
    if (S_Intake.isDiverterDown()) {
      state = 2;
    }
    if (S_Arm.MoveArm(Constants.ArmConstants.restPosition) > 2) { //don't move the note to the arm if the arm isn't in the right spot
      state = -1; 
      isDone = true;
      //TODO blink LEDs red or something
     
    }
     S_Shooter.setShooterAngle(74);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
    case 0://pull note up far enough for diverter to fall down
    //TODO add a check to see if arm is near position already, if not we need an intermediate position
      S_Arm.MoveArm(Constants.ArmConstants.restPosition);
      
      if (S_Intake.isDiverterDown()) {
        S_Intake.stopIntakeMotors();
      }
      else {
        S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerIntakeSpeed);
        S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.indexerIntakeSpeed);
      }
      if (S_Intake.isDiverterDown() && S_Arm.MoveArm(Constants.ArmConstants.restPosition) < 4) {
        state ++;
      }
        
      break;
    case 1:
      pause ++;
      S_Intake.stopIntakeMotors();
      if (pause > 1) {
        state ++;
      }
      break;

    case 2:
      S_Shooter.setRightAndLeftRPM(200, 200);
      S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerOuttakeSpeed);
      S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.indexerOuttakeSpeed);
      S_Arm.setRollerOutputPercent(Constants.ArmConstants.armIntakeSpeed);
      S_Intake.setIndexerPecentOutput(-Constants.IntakeConstants.indexRollerShootSpeed/2);
       
      if (S_Arm.getHasNote()) {
        S_Arm.setRollerPositionToZero();
        S_Arm.moveArmRollers(Constants.ArmConstants.intakeRollerPosition);
        isDone = true;
       }
      break;
    }
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
    return isDone;
  }
}
