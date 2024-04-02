// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;


public class Shooter extends SubsystemBase {
  private CANSparkFlex shooterRightMotor;
  private CANSparkFlex shooterLeftMotor;
  private CANSparkFlex shooterAngleMotor;
  private SparkPIDController angleMotorController;
  private SparkPIDController rightMotorController;
  private SparkPIDController leftMotorController;
 // private final DutyCycleEncoder shooterAngleAbsoluteEncoder;
  private RelativeEncoder integratedRightMotorEncoder;
  private RelativeEncoder integratedLeftMotorEncoder;
  private RelativeEncoder integratedAngleMotorEncoder;
  private DutyCycleEncoder shooterAngleAbsolute;
  
  double shooterRightRPMTarget = 0;
  double shooterLeftRPMTarget = 0;

  double shooterAngleTarget = Constants.ShooterConstants.shooterStartingAngle;
  double shooterCalcAdjustment =0 ;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRightMotorCanID, MotorType.kBrushless);
    rightMotorController = shooterRightMotor.getPIDController();
    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeftMotorCanID, MotorType.kBrushless);
    leftMotorController = shooterLeftMotor.getPIDController();
    shooterAngleMotor = new CANSparkFlex(Constants.ShooterConstants.shooterAngleMotorCanID, MotorType.kBrushless);
    angleMotorController = shooterAngleMotor.getPIDController();

    shooterAngleAbsolute = new DutyCycleEncoder(Constants.ShooterConstants.shooterAngleEncoderAbsoluteID);
    //shooterAngleAbsoluteEncoder = new DutyCycleEncoder(Constants.ShooterConstants.shooterAngleEncoderAbsoluteID);
    setShooterPIDFToSpinUp();
    angleMotorController.setP(Constants.ShooterConstants.angleP);
    angleMotorController.setI(Constants.ShooterConstants.angleI);
    angleMotorController.setD(Constants.ShooterConstants.angleD);
    angleMotorController.setFF(Constants.ShooterConstants.angleFF);
    integratedRightMotorEncoder = shooterRightMotor.getEncoder();
    integratedLeftMotorEncoder = shooterLeftMotor.getEncoder();
    integratedAngleMotorEncoder = shooterAngleMotor.getEncoder();
    integratedAngleMotorEncoder.setPositionConversionFactor(Constants.ShooterConstants.angleAngleConversionFactor);

    angleMotorController.setOutputRange(-0.6, 0.6);
    shooterAngleMotor.setSmartCurrentLimit(40);
    shooterRightMotor.setSmartCurrentLimit(80);
    shooterLeftMotor.setSmartCurrentLimit(80);
    integratedAngleMotorEncoder.setPosition(Constants.ShooterConstants.shooterStartingAngle);

    shooterRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    shooterRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    shooterRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    shooterLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    shooterLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
  
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
    // SmartDashboard.putNumber( 
    //               "Shooter Absolute Angle ", shooterAngleAbsoluteEncoder.getAbsolutePosition());  

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
    shooterRightRPMTarget = rightRPM;
    shooterLeftRPMTarget = leftRPM;
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
  public void resetShooterAngle() {
    //integratedAngleMotorEncoder.setPosition(Constants.ShooterConstants.shooterStartingAngle);
    integratedAngleMotorEncoder.setPosition(shooterAngleAbsolute.getAbsolutePosition() * 360 - Constants.ShooterConstants.angleAbsoluteOffset);
  }
  public void setShooterPIDFToSpinUp() {
    rightMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    rightMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    rightMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
    rightMotorController.setFF(Constants.ShooterConstants.shooter_SpinUp_FF);
    leftMotorController.setP(Constants.ShooterConstants.shooter_SpinUp_P);
    leftMotorController.setI(Constants.ShooterConstants.shooter_SpinUp_I);
    leftMotorController.setD(Constants.ShooterConstants.shooter_SpinUp_D);
    leftMotorController.setFF(Constants.ShooterConstants.shooter_SpinUp_FF);
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
    if (angle > Constants.ShooterConstants.maxShooterAngle) {
      angle = Constants.ShooterConstants.maxShooterAngle;
    }
    else if (angle < Constants.ShooterConstants.minShooterAngle) {
      angle = Constants.ShooterConstants.minShooterAngle;
    }
      angleMotorController.setReference(angle, com.revrobotics.CANSparkFlex.ControlType.kPosition);
      return Math.abs(angle - getAnglePosition());
  }
  public void stopShooterMotor() {
    shooterRightMotor.disable();
    shooterLeftMotor.disable();
  }

  public void stopAngleMotors() {
    shooterAngleMotor.disable();
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
  public void increaseShooterAngle() {
    shooterAngleTarget++;
    if (shooterAngleTarget > 94) {
      shooterAngleTarget = 94;
    }
    setShooterAngle(shooterAngleTarget);
  }
  public void decreaseShooterAngle() {
    shooterAngleTarget--;
    if (shooterAngleTarget < 64) {
      shooterAngleTarget = 64;
    }
    setShooterAngle(shooterAngleTarget);
  }
  public void higherShooterCalcAdjustment() {
    shooterCalcAdjustment -= 0.5;
  }
  public void lowerShooterCalcAdjustment() {
    shooterCalcAdjustment += 0.5;
  }
  public double calculateShooterAngle(double targetY) {
    //*First shooter angle calcs as of March 1st 2024 */
    //shooterAngleTarget = 0.0144 * Math.pow(targetY ,2) - 1.3578 * targetY + 71.7528;
   // shooterAngleTarget = 0.0049 * Math.pow(targetY, 3) + 0.0628 * Math.pow(targetY, 2) -1.6032 * targetY + 70.284;
   // shooterAngleTarget = 0.0053 * Math.pow(targetY, 3) + 0.0725 * Math.pow(targetY, 2) -1.6548 * targetY + 70.1025;
    //shooterAngleTarget = 0.0009 * Math.pow(targetY, 4) + 0.0176 * Math.pow(targetY, 3) + 0.0315 * Math.pow(targetY, 2) -2.2287 * targetY + 71.6418; //70.1218
   
   
   /*New shooter calcs March 9th 2024 */
   shooterAngleTarget = 0.0341 * Math.pow(targetY ,2) - 1.237 * targetY + 70.0036 + shooterCalcAdjustment;
    if (shooterAngleTarget < Constants.ShooterConstants.minShooterAngle) {
      shooterAngleTarget = Constants.ShooterConstants.minShooterAngle;
    }
    else if (shooterAngleTarget > Constants.ShooterConstants.maxShooterAngle) {
      shooterAngleTarget = Constants.ShooterConstants.maxShooterAngle;
    }
    return shooterAngleTarget;
  }
  public double getLeftTargetVelocity() {
    return shooterLeftRPMTarget;
  }
  public double getRightTargetVelocity() {
    return shooterRightRPMTarget;
  }
}
