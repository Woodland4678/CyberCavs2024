// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
  public Climber(PowerDistribution PHD) {
    climberMainMotor = new CANSparkFlex(Constants.ClimberConstants.ClimberMainMotorCanID, MotorType.kBrushless);
    climberController = climberMainMotor.getPIDController();
    climberDown = new DigitalInput(Constants.ClimberConstants.climberDownChannel);
    climberController.setP(Constants.ClimberConstants.climberP);
    climberController.setI(Constants.ClimberConstants.climberI);
    climberController.setD(Constants.ClimberConstants.climberD);
    integratedClimberEncoder = climberMainMotor.getEncoder();
    powerDistribution = PHD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
      "Climber isClimberDown Sensor", isClimberDown());
    // This method will be called once per scheduler run
  }
  public void setClimberSpeed(int speed) {
    climberController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public boolean isClimberDown() {
    return !climberDown.get();
  }

  public void disengageLock() { // toggles the switchable channel on to unlock the climber. 
    powerDistribution.setSwitchableChannel(true);
  }

  public void engageLock() { // toggles the switchable channel off to lock climber. 
    powerDistribution.setSwitchableChannel(false);
  }
}
