// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SubwooferShot extends Command {
  Shooter S_Shooter;
  Intake S_Intake;
  boolean isShooting = false;
  boolean hasShot = false;
  int hasShotCnt = 0;
  boolean isDone = false;
  boolean isAuto;
  /** Creates a new SubwooferShot. */
  public SubwooferShot(Shooter S_Shooter, Intake S_Intake, boolean isAuto) {
    this.S_Shooter = S_Shooter;
    this.S_Intake = S_Intake;
    addRequirements(S_Shooter, S_Intake);
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isShooting = false;
    hasShot = false;
    hasShotCnt = 0;
    isDone = false;
    S_Shooter.setShooterAngle(Constants.ShooterConstants.subwooferShotAngle);
    S_Shooter.setRightAndLeftRPM(Constants.ShooterConstants.subwooferShotRightRPM, Constants.ShooterConstants.subwooferShotLeftRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (S_Shooter.setShooterAngle(Constants.ShooterConstants.subwooferShotAngle) < Constants.ShooterConstants.subwooferShotAngleTolerance && 
      Math.abs(S_Shooter.getLeftVelocity() - Constants.ShooterConstants.subwooferShotLeftRPM) < Constants.ShooterConstants.subwooferShotRPMTolerance &&
      Math.abs(S_Shooter.getRightVelocity() - Constants.ShooterConstants.subwooferShotRightRPM) < Constants.ShooterConstants.subwooferShotRPMTolerance ) {

       // S_Intake.setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerShootSpeed);
       // S_Intake.setHorizontalPercentOutput(Constants.IntakeConstants.horizontalRollerShootSpeed);
       isShooting = true;
        S_Intake.setRampRollerMotorPercentOutput(Constants.IntakeConstants.rampRollerShooterSpeed);
        S_Intake.setIndexerPecentOutput(Constants.IntakeConstants.indexRollerShootSpeed);
      }
    if (isShooting) {
      if (Math.abs(S_Shooter.getLeftVelocity() - S_Shooter.getLeftTargetVelocity()) > 300) {
        hasShot = true;
      }
    }
    if (hasShot && isAuto) {
      hasShotCnt++;
      if (hasShotCnt > 5) {
        isDone = true;
      }
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Intake.stopIntakeMotors();
    S_Shooter.stopShooterMotor();
    S_Shooter.stopAngleMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
