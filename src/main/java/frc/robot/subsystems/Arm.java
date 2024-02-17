// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armShoulderMotor; //Neo for shoulder
  private CANSparkMax armElbowMotor; //Neo for elbow
  private CANSparkMax armWristMotor;//wrist Neo 550
  private CANSparkMax armRollerMotor; //roller Neo 550
  private SparkPIDController shoulderController;
  private SparkPIDController elbowController;
  private SparkPIDController wristController;
  private SparkPIDController rollerController;
  private final DutyCycleEncoder shoulderAbsolute; // Absoloute Encoder
  private final DutyCycleEncoder elbowAbsolute;
  private RelativeEncoder integratedShoulderEncoder;
  private RelativeEncoder integratedElbowEncoder;
  private RelativeEncoder integratedWristEncoder;
  private RelativeEncoder integratedRollerEncoder;
  private DigitalInput hasNoteSensor;

  ArmPosition currentArmPosition = Constants.ArmConstants.restPosition;
  public Arm() {
    armShoulderMotor = new CANSparkMax(Constants.ArmConstants.armShoulderMotorCanID, MotorType.kBrushless);
    shoulderController = armShoulderMotor.getPIDController();
    armElbowMotor = new CANSparkMax(Constants.ArmConstants.armElbowMotorCanID, MotorType.kBrushless);
    elbowController = armElbowMotor.getPIDController();
    armWristMotor = new CANSparkMax(Constants.ArmConstants.armWristMotorCanID, MotorType.kBrushless);
    wristController = armWristMotor.getPIDController();
    armRollerMotor = new CANSparkMax(Constants.ArmConstants.armRollerMotorCanID, MotorType.kBrushless);
    rollerController = armRollerMotor.getPIDController();
    shoulderAbsolute = new DutyCycleEncoder(Constants.ArmConstants.shoulderEncoderAbsoluteID);
    elbowAbsolute = new DutyCycleEncoder(Constants.ArmConstants.elbowEncoderAbsoluteID);
    shoulderController.setP(Constants.ArmConstants.shoudlerP);
    shoulderController.setI(Constants.ArmConstants.shoulderI);
    shoulderController.setD(Constants.ArmConstants.shoulderD);
    elbowController.setP(Constants.ArmConstants.elbowP);
    elbowController.setI(Constants.ArmConstants.elbowI);
    elbowController.setD(Constants.ArmConstants.elbowD);
    wristController.setP(Constants.ArmConstants.wristP);
    wristController.setI(Constants.ArmConstants.wristI);
    wristController.setD(Constants.ArmConstants.wristD);
    rollerController.setP(0.1);
    rollerController.setI(0);
    rollerController.setD(0);
    integratedShoulderEncoder = armShoulderMotor.getEncoder();
    integratedElbowEncoder = armElbowMotor.getEncoder();
    integratedWristEncoder = armWristMotor.getEncoder();
    integratedRollerEncoder = armRollerMotor.getEncoder();
    integratedShoulderEncoder.setPositionConversionFactor(Constants.ArmConstants.shoulderAngleConversionFactor);
    integratedElbowEncoder.setPositionConversionFactor(Constants.ArmConstants.elbowAngleConversionFactor);
    integratedWristEncoder.setPositionConversionFactor(Constants.ArmConstants.wristAngleConversionFactor);
    shoulderController.setOutputRange(-0.1, 0.1);
    elbowController.setOutputRange(-0.1, 0.1);
    hasNoteSensor = new DigitalInput(Constants.ArmConstants.hasNoteSensor);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( 
                  "Arm Elbow Angle", getCurrentElbowPosition());
    SmartDashboard.putNumber( 
                  "Arm Elbow Absolute", elbowAbsolute.getAbsolutePosition() * 360);
    SmartDashboard.putNumber( 
                  "Arm Shoulder Absolute", (shoulderAbsolute.getAbsolutePosition()) * 360);
    SmartDashboard.putNumber( 
                  "Arm Shoulder Angle",90 - getCurrentShoulderPosition());    
    SmartDashboard.putNumber( 
                  "Arm X Position", getCurrentXPosition());
    SmartDashboard.putNumber( 
                  "Arm Y Position", getCurrentYPosition());    
    SmartDashboard.putNumber( 
                  "Arm Wrist Position", getCurrentWristPosition());
    SmartDashboard.putNumber(
                  "Arm Elbow Angle", calcElbowAngle(getCurrentXPosition(), getCurrentYPosition())); 
    SmartDashboard.putNumber(
                  "Arm Shoulder Angle", calcShoulderAngle(getCurrentXPosition(), getCurrentYPosition(), getCurrentElbowPosition()));   

  }

  public void moveToAngle(double shoulderAngle, double elbowAngle) {
    shoulderController.setReference((90 - shoulderAngle) , com.revrobotics.CANSparkMax.ControlType.kPosition); //90 - inverse calc
    elbowController.setReference(-elbowAngle, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }
  public double getCurrentElbowPosition() {
    return integratedElbowEncoder.getPosition();
  }
  public double getCurrentShoulderPosition() {
    return integratedShoulderEncoder.getPosition();
  }
  public double getCurrentWristPosition() {
    return integratedWristEncoder.getPosition();
  }
  public double getCurrentXPosition() {
    double shoulderLength = Constants.ArmConstants.shoulderLength;
    double elbowLength = Constants.ArmConstants.elbowLength;
    double shoulderAngle = 90 - integratedShoulderEncoder.getPosition();
    double elbowAngle = integratedElbowEncoder.getPosition();
    return shoulderLength * Math.cos(Math.toRadians(shoulderAngle)) + elbowLength * Math.cos(Math.toRadians((shoulderAngle) - elbowAngle));
  }
  public double getCurrentYPosition() {
    double shoulderLength = Constants.ArmConstants.shoulderLength;
    double elbowLength = Constants.ArmConstants.elbowLength;
    double shoulderAngle = 90 - integratedShoulderEncoder.getPosition();
    double elbowAngle = integratedElbowEncoder.getPosition();
    return (shoulderLength * Math.sin(Math.toRadians(shoulderAngle)) + elbowLength * Math.sin(Math.toRadians((shoulderAngle) - elbowAngle)));
  }

  public boolean getHasNote(){
    return hasNoteSensor.get()
  }
  
  public void setElbowPIDF(double p, double i, double d, double iz, double ff) {
    elbowController.setP(p);
    elbowController.setI(i);
    elbowController.setD(d);
    elbowController.setIZone(iz);
    elbowController.setFF(ff);
  }
  public void setShoulderPIDF(double p, double i, double d, double iz, double ff) {
    shoulderController.setP(p);
    shoulderController.setI(i);
    shoulderController.setD(d);
    shoulderController.setIZone(iz);
    shoulderController.setFF(ff);
  }
  private double calcShoulderAngle(double x, double y, double calculatedElbowAngle) {
    return Math.toDegrees(Math.atan(y / x) - Math.atan((Constants.ArmConstants.elbowLength * Math.sin(Math.toRadians(calculatedElbowAngle))) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(Math.toRadians(calculatedElbowAngle)))));
  }
  private double calcElbowAngle(double x, double y) {
    return Math.toDegrees(- Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength)));
  }
  public double MoveArm(ArmPosition targetPos) {
    double currentX = getCurrentXPosition();
    double currentY = getCurrentYPosition();
    if (targetPos != currentArmPosition) {
      double elbowAngle = calcElbowAngle(targetPos.xTarget, targetPos.yTarget);
      double shoulderAngle = calcShoulderAngle(targetPos.xTarget, targetPos.yTarget, elbowAngle);
      if (shoulderAngle >= 45 && shoulderAngle <= 138 && elbowAngle <= 0 && elbowAngle >= -175 ) { // 2023 values, change once robot done
        moveToAngle(shoulderAngle, elbowAngle);
      }
      if (currentX > 17 || targetPos.wristPitchTarget > -30 || currentY > 25) {
        wristController.setReference(targetPos.wristPitchTarget, com.revrobotics.CANSparkMax.ControlType.kPosition);
      }
      else {
        armWristMotor.stopMotor();
      }
      currentArmPosition = targetPos;
    }
    double error = Math.sqrt((Math.pow(currentX - targetPos.xTarget,2)) + (Math.pow(currentY - targetPos.yTarget, 2))); //returns distance to target
    return error;
  }
  public void setRollerOutputPercent(double percent){
    armRollerMotor.set(percent);
  }
}
