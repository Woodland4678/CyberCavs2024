// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterRightMotor;
  private CANSparkFlex shooterLeftMotor;
  private CANSparkFlex shooterAngleMotor;
  private SparkPIDController angleMotorController;
  private SparkPIDController rightMotorController;
  private SparkPIDController leftMotorController;
  private final DutyCycleEncoder shooterAngleAbsoluteEncoder;
  private RelativeEncoder integratedRightMotorEncoder;
  private RelativeEncoder integratedLeftMotorEncoder;
  private RelativeEncoder integratedAngleMotorEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRightMotorCanID, MotorType.kBrushless);
    rightMotorController = shooterRightMotor.getPIDController();
    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeftMotorCanID, MotorType.kBrushless);
    leftMotorController = shooterLeftMotor.getPIDController();
    shooterAngleMotor = new CANSparkFlex(Constants.ShooterConstants.shooterAngleMotorCanID, MotorType.kBrushless);
    angleMotorController = shooterAngleMotor.getPIDController();
    shooterAngleAbsoluteEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterAngleEncoderAbsoluteID);
    rightMotorController.setP(Constants.ShooterConstants.shooterP);
    rightMotorController.setI(Constants.ShooterConstants.shooterI);
    rightMotorController.setD(Constants.ShooterConstants.shooterD);
    leftMotorController.setP(Constants.ShooterConstants.shooterP);
    leftMotorController.setI(Constants.ShooterConstants.shooterI);
    leftMotorController.setD(Constants.ShooterConstants.shooterD);
    angleMotorController.setP(Constants.ShooterConstants.angleP);
    angleMotorController.setI(Constants.ShooterConstants.angleI);
    angleMotorController.setD(Constants.ShooterConstants.angleD);
    integratedRightMotorEncoder = shooterRightMotor.getEncoder();
    integratedLeftMotorEncoder = shooterLeftMotor.getEncoder();
    integratedAngleMotorEncoder = shooterAngleMotor.getEncoder();
    integratedAngleMotorEncoder.setPositionConversionFactor(Constants.ShooterConstants.angleAngleConversionFactor);

    angleMotorController.setOutputRange(-0.1, 0.1);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( 
                  "Shooter Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber( 
                  "Shooter Right Velocity", getRightVelocity());  
    SmartDashboard.putNumber( 
                  "Shooter Current Angle ", getAnglePosition());  
    SmartDashboard.putNumber( 
                  "Shooter Absolute Angle ", shooterAngleAbsoluteEncoder.getAbsolutePosition());  
  }
  public void shoot(double rightRPM, double leftRPM) {
    rightMotorController.setReference(rightRPM,  com.revrobotics.CANSparkMax.ControlType.kVelocity);
    leftMotorController.setReference(leftRPM, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public double getRightVelocity() {
    return integratedRightMotorEncoder.getVelocity();
  }
  public double getLeftVelocity() {
    return integratedLeftMotorEncoder.getVelocity();
  }
  public double getAnglePosition() {
    return integratedAngleMotorEncoder.getPosition();
  }
}
