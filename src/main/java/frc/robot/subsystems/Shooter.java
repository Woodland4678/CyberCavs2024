// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  
  double shooterRightRPMTarget = 0;
  double shooterLeftRPMTarget = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRightMotorCanID, MotorType.kBrushless);
    rightMotorController = shooterRightMotor.getPIDController();
    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeftMotorCanID, MotorType.kBrushless);
    leftMotorController = shooterLeftMotor.getPIDController();
    shooterAngleMotor = new CANSparkFlex(Constants.ShooterConstants.shooterAngleMotorCanID, MotorType.kBrushless);
    angleMotorController = shooterAngleMotor.getPIDController();
    shooterAngleAbsoluteEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterAngleEncoderAbsoluteID);
    setShooterPIDFToSpinUp();
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

    // if (Math.abs(integratedRightMotorEncoder.getVelocity() - shooterRightRPMTarget) > 100) {
    //   rightMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    //   rightMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    //   rightMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
    // }
    // else {
    //   rightMotorController.setP(Constants.ShooterConstants.shooter_Maintain_P);
    //   rightMotorController.setI(Constants.ShooterConstants.shooter_Maintain_I);
    //   rightMotorController.setD(Constants.ShooterConstants.shooter_Maintain_D);
    // }
    // if (Math.abs(integratedLeftMotorEncoder.getVelocity() - shooterLeftRPMTarget) > 100) {
    //   leftMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    //   leftMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    //   leftMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
    // }
    // else {
    //   leftMotorController.setP(Constants.ShooterConstants.shooter_Maintain_P);
    //   leftMotorController.setI(Constants.ShooterConstants.shooter_Maintain_I);
    //   leftMotorController.setD(Constants.ShooterConstants.shooter_Maintain_D);
    // }
  }
  public void setRightAndLeftRPM(double rightRPM, double leftRPM) {
    
    rightMotorController.setReference(rightRPM,  com.revrobotics.CANSparkFlex.ControlType.kVelocity);
    leftMotorController.setReference(leftRPM, com.revrobotics.CANSparkFlex.ControlType.kVelocity);
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
  public void setShooterPIDFToSpinUp() {
    rightMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    rightMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    rightMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
    leftMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    leftMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    leftMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
  }
  public void setShooterPIDFToMaintain() {
    rightMotorController.setP(Constants.ShooterConstants.shooter_Maintain_P);
    rightMotorController.setI(Constants.ShooterConstants.shooter_Maintain_I);
    rightMotorController.setD(Constants.ShooterConstants.shooter_Maintain_D);
    leftMotorController.setP(Constants.ShooterConstants.shooter_Maintain_P);
    leftMotorController.setI(Constants.ShooterConstants.shooter_Maintain_I);
    leftMotorController.setD(Constants.ShooterConstants.shooter_Maintain_D);
  }
  public void setShooterPIDF(double p, double i, double d, double iz, double ff) {
    rightMotorController.setP(p);
    rightMotorController.setI(i);
    rightMotorController.setD(d);
    rightMotorController.setIZone(iz);
    rightMotorController.setFF(ff);
    leftMotorController.setP(p);
    leftMotorController.setI(i);
    leftMotorController.setD(d);
    leftMotorController.setIZone(iz);
    leftMotorController.setFF(ff);
  }
  public double setShooterAngle(double angle) {
      angleMotorController.setReference(angle, com.revrobotics.CANSparkFlex.ControlType.kPosition);
      return Math.abs(angle - getAnglePosition());
  }
  public void stopShooterMotor() {
    shooterRightMotor.disable();
    shooterLeftMotor.disable();
  }
  public void setShooterAnglePIDF(double p, double i, double d, double iz, double ff) {
    angleMotorController.setP(p);
    angleMotorController.setI(i);
    angleMotorController.setD(d);
    angleMotorController.setIZone(iz);
    angleMotorController.setFF(ff);
  }
   public void setShooterMotorsPIDF(double p, double i, double d, double iz, double ff) {
    rightMotorController.setP(p);
    rightMotorController.setI(i);
    rightMotorController.setD(d);
    rightMotorController.setIZone(iz);
    rightMotorController.setFF(ff);

    leftMotorController.setP(p);
    leftMotorController.setI(i);
    leftMotorController.setD(d);
    leftMotorController.setIZone(iz);
    leftMotorController.setFF(ff);
  }
}
