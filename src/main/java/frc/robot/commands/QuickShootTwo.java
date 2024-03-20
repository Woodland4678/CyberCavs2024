// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class QuickShootTwo extends Command {
  Shooter S_Shooter;
  Intake S_Intake;
  SwerveSubsystem S_Swerve;
  boolean isDone = false;
  int state = 0;
  int count = 0;
  /** Creates a new QuickShootTwo. */
  public QuickShootTwo(Shooter S_Shooter, Intake S_Intake, SwerveSubsystem S_Swerve) {
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    this.S_Swerve = S_Swerve;
    addRequirements(S_Shooter, S_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    state = 0;
    count = 0;
    S_Shooter.setShooterAngle(Constants.ShooterConstants.AmpSideQuickShootTwoAngle);
    S_Shooter.setRightAndLeftRPM(-4400, -3400);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //S_Shooter.setShooterAngle(S_Shooter.calculateShooterAngle(S_Swerve.getAprilTagY()));
    S_Intake.setHorizontalPercentOutput(Constants.IntakeConstants.horizontalRollerIntakeSpeed);
    S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerIntakeSpeed);
    S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
    S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
    SmartDashboard.putNumber("QuickShooterTwo_State", state);
    switch (state) {
      case 0: //first note goes through
        if (!S_Intake.isNoteOnRamp()) {
          state++;
        }
        // if (Math.abs(S_Shooter.getLeftTargetVelocity() - S_Shooter.getLeftVelocity()) > 300) {
        //   state++;
        // }
        break;
    
      case 1: //spin back up
        // if (Math.abs(S_Shooter.getLeftTargetVelocity() - S_Shooter.getLeftVelocity()) < 100) {
        //   state++;
           
        if (S_Intake.isNoteOnRamp()) {
          state++;
          S_Shooter.setShooterAngle(86);
        }
        break;
      case 2: //second note goes through
        // if (Math.abs(S_Shooter.getLeftTargetVelocity() - S_Shooter.getLeftVelocity()) > 300) {
        //   state++;
        // }
        if (!S_Intake.isNoteOnRamp()) {
          state++;
        }
        break;
      case 3:
        count++;
        if (count >10) {
          isDone = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();
    //S_Shooter.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
