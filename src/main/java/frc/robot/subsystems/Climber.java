// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkFlex climberMainMotor; //Vortex drives up down
  private SparkPIDController climberController;
  private DigitalInput climberDown;
  private RelativeEncoder integratedClimberEncoder;
  private PowerDistribution powerDistribution;
  boolean isClimberLocked = true;
  public Climber(PowerDistribution PHD) {
    climberMainMotor = new CANSparkFlex(Constants.ClimberConstants.ClimberMainMotorCanID, MotorType.kBrushless);
    climberController = climberMainMotor.getPIDController();
    climberDown = new DigitalInput(Constants.ClimberConstants.climberDownChannel);
    climberController.setP(Constants.ClimberConstants.climberP);
    climberController.setI(Constants.ClimberConstants.climberI);
    climberController.setD(Constants.ClimberConstants.climberD);
    integratedClimberEncoder = climberMainMotor.getEncoder();
    powerDistribution = PHD;
    climberController.setOutputRange(-1, 1);
    climberMainMotor.setSmartCurrentLimit(80);
   // climberMainMotor.setIdleMode(mode.brake);
    climberMainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    climberMainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    climberMainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    climberMainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
      "Climber isClimberDown Sensor", isClimberDown());

    SmartDashboard.putNumber("Climber Position", integratedClimberEncoder.getPosition());
    // This method will be called once per scheduler run
  }
  public void setClimberSpeed(int speed) {
    if (integratedClimberEncoder.getPosition() < Constants.ClimberConstants.minClimberPosition) {
      speed = 0;
    }
    climberController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public boolean isClimberDown() {
    return !climberDown.get();
  }

  public void disengageLock() { // toggles the switchable channel on to unlock the climber. 
    powerDistribution.setSwitchableChannel(true);
    isClimberLocked = false;
  }

  public void engageLock() { // toggles the switchable channel off to lock climber. 
    powerDistribution.setSwitchableChannel(false);
    isClimberLocked = true;
  }
  public void moveClimberToPosition(double pos) {
    if (isClimberLocked == false) {
      climberController.setReference(pos, com.revrobotics.CANSparkFlex.ControlType.kPosition);
    }
  }
  // public void setClimberSpeed(double speed) {
  //   if (!isClimberLocked) {
  //     if (integratedClimberEncoder.getPosition() > -15) {
  //       climberController.setReference(-5, com.revrobotics.CANSparkFlex.ControlType.kPosition);
  //     }
  //     else if (!isClimberDown()) {
  //       climberMainMotor.set(speed); //positive to bring climber down/climb
  //     }
  //     if (isClimberDown()) {
  //       stopClimber();
  //     }
  //   }
  // }
  public void setClimberSpeed(double speed) {
    
    if (isClimberDown()) {
      stopClimber();
    }
    else {
      climberMainMotor.set(speed);
    }
  }
  public void stopClimber() {
    climberMainMotor.disable();
  }
  public void setPosition(double pos) {
    integratedClimberEncoder.setPosition(pos);
  }
  public void setClimberPIDF(double p, double i, double d, double iz, double ff) {
    climberController.setP(p);
    climberController.setI(i);
    climberController.setD(d);
    climberController.setIZone(iz);
    climberController.setFF(ff);
  }
  public double getClimberPosition() {
    return integratedClimberEncoder.getPosition();
  }
}
