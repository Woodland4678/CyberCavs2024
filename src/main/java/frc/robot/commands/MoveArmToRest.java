// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class MoveArmToRest extends Command {
  /** Creates a new MoveArmToRest. */
  Arm S_Arm;
  int moveToAmpState = 0;
  int count = 0;
  Shooter S_Shooter;
  public MoveArmToRest(Arm S_Arm, Shooter S_Shooter) {
    this.S_Arm = S_Arm;
    this.S_Shooter = S_Shooter;
    addRequirements(S_Arm, S_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveToAmpState = 0;
    if (S_Arm.getCurrentXPosition() < 5 && S_Arm.getCurrentYPosition() < 5.5) {
      moveToAmpState = 2;
      S_Arm.MoveArm(Constants.ArmConstants.restPosition);
    }
    S_Shooter.setShooterAngle(75);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (moveToAmpState) {
      case 0:
        if (S_Arm.MoveArm(Constants.ArmConstants.AmpIntermediatePos2) < 3) {
          count ++;
          if (count > 5) {
            moveToAmpState ++;
            count = 0;
          }
        }
        else {
          count = 0;
        }   
        break;     
      case 1:
        S_Arm.moveToAngle(Constants.ArmConstants.AmpIntermediatePos1.xTarget, Constants.ArmConstants.AmpIntermediatePos1.yTarget);
        if (S_Arm.getShoulderAngleError() < 4 && S_Arm.getElbowAngleError() < 4) {
          count ++;
          if (count > 3) {
            moveToAmpState ++;
            count = 0;
            S_Shooter.setShooterAngle(Constants.ShooterConstants.shooterStartingAngle);
          }
        }
        else {count = 0;} 
        break;
      case 2:       
        if (S_Arm.MoveArm(Constants.ArmConstants.restPosition) < 3) {
          count ++;
          if (count > 5) {
            //S_Arm.stopElbowMotor();
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
