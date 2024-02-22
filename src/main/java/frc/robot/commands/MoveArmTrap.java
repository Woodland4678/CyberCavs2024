// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants;

public class MoveArmTrap extends Command {
  Arm S_Arm;
  ArmPosition targetPos;
  /** Creates a new MoveArmTrap. */
  public MoveArmTrap(Arm S_Arm) {
    this.S_Arm = S_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(S_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //targetPos = Constants.ArmConstants.trapScoring;
    targetPos = Constants.ArmConstants.testPos1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    S_Arm.MoveArm(targetPos);
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
