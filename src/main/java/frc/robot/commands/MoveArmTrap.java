// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants;

public class MoveArmTrap extends Command {
  Arm S_Arm;
  Shooter S_Shooter;
  ArmPosition targetPos;
  int moveToTrapState = 0;
  CommandXboxController operatorController;
  int count = 0;
  /** Creates a new MoveArmTrap. */
  public MoveArmTrap(Arm S_Arm, Shooter S_Shooter, CommandXboxController operatorController) {
    this.S_Arm = S_Arm;
    this.S_Shooter = S_Shooter;
    this.operatorController = operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(S_Arm, S_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //targetPos = Constants.ArmConstants.trapScoring;
    //targetPos = Constants.ArmConstants.testPos1;
    moveToTrapState = 0;
    count = 0;
    S_Shooter.setShooterAngle(85);
    //S_Arm.setArmRollers(18);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (moveToTrapState) {
      case 0:
       // S_Arm.MoveArm(Constants.ArmConstants.AmpIntermediatePos1);
       S_Arm.moveToAngle(Constants.ArmConstants.AmpIntermediatePos1.xTarget, Constants.ArmConstants.AmpIntermediatePos1.yTarget);
        if (S_Arm.getShoulderAngleError() < 4.5) {
          count ++;
          if (count > 5) {
            moveToTrapState ++;
            count = 0;
          }
        }
        else {count = 0;}   
        break;     
      case 1:
        if (S_Arm.MoveArm(Constants.ArmConstants.trapIntermediatePos2) < 4) {
          count ++;
          if (count > 3) {
            moveToTrapState ++;
            count = 0;
          }
        }
        else {count = 0;} 
        break;
      case 2:
       S_Arm.moveToAngle(Constants.ArmConstants.trapScoring.xTarget, Constants.ArmConstants.trapScoring.yTarget);
       S_Arm.moveWristToAngle(Constants.ArmConstants.trapScoring.wristPitchTarget);
       moveToTrapState++;
        break;
      case 3:
        if (operatorController.leftStick().getAsBoolean()) {
          moveToTrapState++;
        }
       
      break;
      case 4:
        if (operatorController.getLeftY() > 0.5) {
          S_Arm.setWristPercentOutput(0.07);
        }
        else if (operatorController.getLeftY() < -0.5) {
          S_Arm.setWristPercentOutput(-0.07);
        }
        else {
          S_Arm.maintainWristPos();
        }
      break;
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
