// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants;

public class MoveArmAmp extends Command {
  Arm S_Arm;
  ArmPosition targetPos;
  int moveToAmpState = 0;
  int count = 0;
  /** Creates a new MoveArm. */
  public MoveArmAmp(Arm S_Arm) {
    this.S_Arm = S_Arm;
    addRequirements(S_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveToAmpState = 0;
    count = 0;
    //targetPos = Constants.ArmConstants.ampScoring;
    //targetPos = Constants.ArmConstants.testPos; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Move To Amp State", moveToAmpState);
    switch (moveToAmpState) {
      case 0:
       // S_Arm.MoveArm(Constants.ArmConstants.AmpIntermediatePos1);
       S_Arm.moveToAngle(Constants.ArmConstants.AmpIntermediatePos1.xTarget, Constants.ArmConstants.AmpIntermediatePos1.yTarget);
        if (S_Arm.getShoulderAngleError() < 5) {
          count ++;
          if (count > 3) {
            moveToAmpState ++;
            count = 0;
          }
        }
        else {count = 0;}   
        break;     
      case 1:
        if (S_Arm.MoveArm(Constants.ArmConstants.AmpIntermediatePos2) < 9) {
          count ++;
          if (count > 3) {
            moveToAmpState ++;
            count = 0;
          }
        }
        else {count = 0;} 
        break;
      case 2:
        if (S_Arm.MoveArm(Constants.ArmConstants.ampScoring) < 4) {
          count ++;
          if (count > 5) {
            moveToAmpState ++;
          }
        }
        else {count = 0;} 
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
