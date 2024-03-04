// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax intakeVerticalRoller; //Neo550 vertical roller
  private CANSparkMax intakeHorizontalRoller; //Neo horizontal roller
  private CANSparkMax rampRoller; //Neo550 passing to roller
  private CANSparkMax indexer;
  private SparkPIDController verticalController;
  private SparkPIDController horizontalController;
  private SparkPIDController rampRollerController;
  private SparkPIDController indexerController;
  private DigitalInput pieceAquired; // Sensor after horizontal roller
  private DigitalInput noteOnRamp; //sensor if the note is on ramp
  private AnalogInput diverter;
  //private DigitalInput pieceDeparted; 
  private RelativeEncoder integratedVerticalRollerEncoder;
  private RelativeEncoder integratedHorizontalRollerEncoder;
  private RelativeEncoder integratedIndexerEncoder;
  private RelativeEncoder integratedRampRollerEncoder;
  int intakeState = 0;
  int count = 0;

  public Intake() {
    intakeVerticalRoller = new CANSparkMax(Constants.IntakeConstants.IntakeVerticalRollerCanID, MotorType.kBrushless);
    verticalController = intakeVerticalRoller.getPIDController();
    intakeHorizontalRoller = new CANSparkMax(Constants.IntakeConstants.IntakeHorizontalRollerCanID, MotorType.kBrushless);
    horizontalController = intakeHorizontalRoller.getPIDController();
    rampRoller = new CANSparkMax(Constants.IntakeConstants.indexerCanID, MotorType.kBrushless);
    rampRollerController = rampRoller.getPIDController();
    pieceAquired = new DigitalInput(Constants.IntakeConstants.pieceAquireChannel);
    diverter = new AnalogInput(Constants.IntakeConstants.diverterUpChannel);
    noteOnRamp = new DigitalInput(Constants.IntakeConstants.noteOnRampSensor);
    indexer = new CANSparkMax(Constants.IntakeConstants.indexerMotorCanID, MotorType.kBrushless);
    indexerController = indexer.getPIDController();
    verticalController.setP(0.1);
    verticalController.setI(0);
    verticalController.setD(0);
    horizontalController.setP(0.1);
    horizontalController.setI(0);
    horizontalController.setD(0);
    rampRollerController.setP(0.1);
    rampRollerController.setI(0);
    rampRollerController.setD(0);
    indexerController.setP(0.1);
    indexerController.setI(0);
    indexerController.setD(0);
    integratedHorizontalRollerEncoder = intakeHorizontalRoller.getEncoder();
    integratedVerticalRollerEncoder = intakeVerticalRoller.getEncoder();
    integratedIndexerEncoder = indexer.getEncoder();
    integratedRampRollerEncoder = rampRoller.getEncoder();
    
    intakeHorizontalRoller.setSmartCurrentLimit(40);
    intakeVerticalRoller.setSmartCurrentLimit(40);
    rampRoller.setSmartCurrentLimit(40);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean( 
                  "Intake Piece Aquired Sensor ", getPieceAquired());  
    SmartDashboard.putNumber( 
                  "Intake diverter sensor ", diverter.getValue());
    SmartDashboard.putBoolean(
                  "Intake Note On Ramp Sensor", isNoteOnRamp());
    
    switch (intakeState) { //constantly running to pull notes fully into robot
      case 0:
        if (getPieceAquired()) {
          setVerticalPercentOutput(Constants.IntakeConstants.verticalRollerIntakeSpeed);
          setHorizontalPercentOutput(Constants.IntakeConstants.horizontalRollerIntakeSpeed);
          setRampRollerMotorPercentOutput(Constants.IntakeConstants.indexerIntakeSpeed);
          intakeState++;
        }
        break;
      case 1:
        if (isNoteOnRamp()) {
          intakeState ++;
          stopHorizontalRoller();
          //stopIntakeMotors();
        }
        break;
      case 2:
          setVerticalPercentOutput(0.55);
          setHorizontalPercentOutput(-0.55);
          setRampRollerMotorPercentOutput(0.55);
          intakeState++;
      break;
      case 3:
        if (isDiverterDown()) {
          stopIntakeMotors();
          intakeState++;
        }
      break;
      case 4:
        if (!isNoteOnRamp()) {
          intakeState = 0;
        }
        break;
        
    }
  }
  public void setIndexerRPM(double speed) {
    indexerController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public void setIndexerPecentOutput (double percent) {
    indexer.set(percent);
  }
  public double getIndexerVelocity() {
    return integratedIndexerEncoder.getVelocity();
  }
  public void setHorizontalRollerRPM(double speed) {
    horizontalController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public void setVerticalRollerRPM(double speed) {
    verticalController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public void setRampRollerRPM(double speed) {
    rampRollerController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public boolean getPieceAquired() {
    return !pieceAquired.get();
  }
  public boolean isDiverterDown() {
    if (diverter.getValue() > 800) {
      return true;
    }
    else {
      return false;
    }
    //return diverterUp.getValue();
  }
  public boolean isNoteOnRamp() {
    return !noteOnRamp.get();
  }
  public void setHorizontalPercentOutput(double percent) {
    intakeHorizontalRoller.set(percent);
  }
  public void setVerticalPercentOutput(double percent) {
    intakeVerticalRoller.set(percent);
  }
  public void setRampRollerMotorPercentOutput(double percent) {
    rampRoller.set(percent);
  }
  public void stopIntakeMotors() {
    rampRoller.stopMotor();
    intakeVerticalRoller.stopMotor();
    intakeHorizontalRoller.stopMotor();
    indexer.stopMotor();
  }
  public void setAllMotorsPercentOutput(double horizontalSpeed, double verticalSpeed, double indexerSpeed, double rampRollerSpeed) {
    setHorizontalPercentOutput(horizontalSpeed);
    setVerticalPercentOutput(verticalSpeed);
    setRampRollerMotorPercentOutput(rampRollerSpeed);
    setIndexerPecentOutput(indexerSpeed);
  }
  public void stopHorizontalRoller() {
    intakeHorizontalRoller.stopMotor();
  }
}
