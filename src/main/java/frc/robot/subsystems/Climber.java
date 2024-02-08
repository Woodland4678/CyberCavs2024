// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkFlex climberMainMotor; //Vortex drives up down
  private SparkPIDController climberController;
  private DigitalInput climberDown;
  private RelativeEncoder integratedClimberEncoder;
  public Climber() {
    climberMainMotor = new CANSparkFlex(Constants.ClimberConstants.ClimberMainMotorCanID, MotorType.kBrushless);
    climberController = climberMainMotor.getPIDController();
    climberDown = new DigitalInput(Constants.ClimberConstants.climberDownChannel);
    climberController.setP(Constants.ClimberConstants.climberP);
    climberController.setI(Constants.ClimberConstants.climberI);
    climberController.setD(Constants.ClimberConstants.climberD);
    integratedClimberEncoder = climberMainMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setClimberSpeed(int speed) {
    climberController.setReference(speed, com.revrobotics.CANSparkMax.ControlType.kVelocity);
  }
  public boolean isClimberDown() {
    return climberDown.get();
  }
}