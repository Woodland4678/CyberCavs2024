// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeVerticalRoller; //Neo550 vertical roller
  private CANSparkMax intakeHorizontalRoller; //Neo horizontal roller
  private CANSparkMax indexer; //Neo550 passing to roller
  private SparkPIDController verticalController;
  private SparkPIDController horizontalController;
  private SparkPIDController indexerController;
  private DigitalInput pieceAquired; // Sensor after horizontal roller
  private DigitalInput diverterUp;
  private DigitalInput pieceDeparted; 
  private RelativeEncoder integratedVerticalRollerEncoder;
  private RelativeEncoder integratedHorizontalRollerEncoder;
  private RelativeEncoder integratedIndexerEncoder;

  public Intake() {
    intakeVerticalRoller = new CANSparkMax(Constants.IntakeConstants.IntakeVerticalRollerCanID, MotorType.kBrushless);
    verticalController = intakeVerticalRoller.getPIDController();
    intakeHorizontalRoller = new CANSparkMax(Constants.IntakeConstants.IntakeHorizontalRollerCanID, MotorType.kBrushless);
    horizontalController = intakeHorizontalRoller.getPIDController();
    indexer = new CANSparkMax(Constants.IntakeConstants.indexerCanID, MotorType.kBrushless);
    indexerController = indexer.getPIDController();
    pieceAquired = new DigitalInput(Constants.IntakeConstants.pieceAquireChannel);
    diverterUp = new DigitalInput(Constants.IntakeConstants.diverterUpChannel);
    pieceDeparted = new DigitalInput(Constants.IntakeConstants.pieceDepartedChannel);
    verticalController.setP(0.1);
    verticalController.setI(0);
    verticalController.setD(0);
    horizontalController.setP(0.1);
    horizontalController.setI(0);
    horizontalController.setD(0);
    indexerController.setP(0.1);
    indexerController.setI(0);
    indexerController.setD(0);
    integratedHorizontalRollerEncoder = intakeHorizontalRoller.getEncoder();
    integratedVerticalRollerEncoder = intakeVerticalRoller.getEncoder();
    integratedIndexerEncoder = indexer.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean( 
                  "Intake Piece Aquired Sensor ", getPieceAquired());  
    SmartDashboard.putBoolean( 
                  "Intake diverter sensor ", getDiverterUp());
    SmartDashboard.putBoolean( 
                  "Intake piece Departed sensor ", getPieceDeparted());    
  }
  
  public void setHorizontalRollerRPM(double speed) {
    horizontalController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public void setVerticalRollerRPM(double speed) {
    verticalController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public void setIndexerRPM(double speed) {
    indexerController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public boolean getPieceAquired() {
    return pieceAquired.get();
  }
  public boolean getDiverterUp() {
    return diverterUp.get();
  }
  public boolean getPieceDeparted() {
    return pieceDeparted.get();
  }
  public void horizontalMotorSetOutput(double percent) {
    intakeHorizontalRoller.set(percent);
  }
  public void verticalMotorSetOutput(double percent) {
    intakeVerticalRoller.set(percent);
  }
  public void indexerMotorSetOutput(double percent) {
    indexer.set(percent);
  }
}
