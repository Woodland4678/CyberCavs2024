// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Shooter extends SubsystemBase {
  private CANSparkMax shooterRightMotor;
  private CANSparkMax shooterLeftMotor;
  private SparkPIDController rightMotorController;
  private SparkPIDController leftMotorController;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterRightMotor = new CANSparkMax(Constants.ShooterConstants.shooterRightMotorCanID, MotorType.kBrushless);
    rightMotorController = shooterRightMotor.getPIDController();
    shooterLeftMotor = new CANSparkMax(Constants.ShooterConstants.shooterLeftMotorCanID, MotorType.kBrushless);
    leftMotorController = shooterLeftMotor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shoot(double rightRPM, double leftRPM) {
    rightMotorController.setReference(rightRPM,  com.revrobotics.CANSparkMax.ControlType.kVelocity);
    leftMotorController.setReference(leftRPM, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
}
