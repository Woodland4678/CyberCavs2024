// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LEDStrip.LEDModes;

import java.io.File;
import java.io.IOException;

import javax.swing.text.AbstractDocument.LeafElement;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  private final SendableChooser<String> pidTuningChooser;
  double tunePID_KP;
  double tunePID_KI;
  double tunePID_KD;
  double tunePID_KIz;
  double tunePID_KFF;

  double tuneShot_KLeftRPM;
  double tuneShot_KRightRPM;
  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private LEDStrip ledstrip;

  public Robot()
  {
    pidTuningChooser = new SendableChooser<String>();
    instance = this;
    pidTuningChooser.addOption("ShooterMotorPIDF", "ShooterMotorPIDF");
    pidTuningChooser.addOption("ElbowPID", "ElbowPID");
    pidTuningChooser.addOption("ShoulderPID", "ShoulderPID");
    pidTuningChooser.addOption("ShooterAnglePID", "ShooterAnglePID");
    pidTuningChooser.addOption("AutoGrabNote_X_PID", "AutoGrabNote_X_PID");
    pidTuningChooser.addOption("AutoGrabNote_Y_PID", "AutoGrabNote_Y_PID");
    pidTuningChooser.addOption("AutoGrabNote_R_PID", "AutoGrabNote_R_PID");
    pidTuningChooser.addOption("AutoAimPID", "AutoAimPID");
    pidTuningChooser.addOption("WristPID", "WristPID");
     pidTuningChooser.addOption("ShooterSpeeds", "ShooterSpeeds");
    pidTuningChooser.setDefaultOption("AutoAimPID", "AutoAimPID");
   
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    m_robotContainer.turnOffClimberLock();
    tunePID_KP =0; 
    tunePID_KI = 0;
    tunePID_KD = 0; 
    tunePID_KIz = 0; 
    tunePID_KFF = 0; 
    tuneShot_KLeftRPM = 0;
    tuneShot_KRightRPM = 0;

    SmartDashboard.putNumber("TunePID P Gain", tunePID_KP);
    SmartDashboard.putNumber("TunePID I Gain", tunePID_KI);
    SmartDashboard.putNumber("TunePID D Gain", tunePID_KD);
    SmartDashboard.putNumber("TunePID I Zone", tunePID_KIz);
    SmartDashboard.putNumber("TunePID Feed Forward", tunePID_KFF);
    SmartDashboard.putNumber("TuneShot Left RPM", tuneShot_KLeftRPM);
    SmartDashboard.putNumber("TuneShot Right RPM", tuneShot_KRightRPM);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.turnOffClimberLock();
    disabledTimer.reset();
    disabledTimer.start();

    ledstrip = LEDStrip.getInstance();
    // ??? test
    // System.out.print("disabledInit() LEDStrip.getInstance() = " + ledstrip.toString());
    for (int index = 0; index < ledstrip.ledBuffer.getLength(); index++){
			ledstrip.ledBuffer.setRGB(index, 0, 0, 255);
		}
    //System.out.print("disabledInit() ledstrip.ledBuffer = " + ledstrip.ledBuffer.toString());
    ledstrip.addressableLED.setData(ledstrip.ledBuffer);
  }

  @Override
  public void disabledPeriodic()
  {
ledstrip = LEDStrip.getInstance();
    // ??? test
    // System.out.print("disabledPeriodic() LEDStrip.getInstance() = " + ledstrip.toString());
    for (int index = 0; index < ledstrip.ledBuffer.getLength(); index++){
			ledstrip.ledBuffer.setRGB(index, 0, 0, 255);
		}
    ledstrip.addressableLED.setData(ledstrip.ledBuffer);

    var diagState = 0; //diagnostic state 
    m_robotContainer.resetArmAngles();
    // 0x01 is first set of LEDs (lower right).  Front right Swerve
    // 0x02 is second set (mid lower right). Shoulder encoder
    // 0x04 is third set (mid upper right). Elbow encoder
    // 0x08 is 4th set (upper right).  Rear right Swerve
    // 0x10 is 5th set (upper left). Rear left Swerve
    // 0x20 is 6th set (mid upper left).  Gyro
    // 0x40 is 7th set (mid lower left).  Limelight
    // 0x80 is 8th set (lower left).  Front left Swerve
    if (m_robotContainer.isElbowReady()){
      diagState += LEDStrip.elbowDiag; 
    }
    if (m_robotContainer.isShoulderReady()){
      diagState += LEDStrip.shoulderDiag; 
    }
    if (m_robotContainer.isGyroReady()){
      diagState += LEDStrip.gyroDiag; 
    }
    if (m_robotContainer.isLimelightReady()){
      diagState += LEDStrip.limelightDiag; 
    }
    if (m_robotContainer.isAprilTagCameraReady()){
      diagState += LEDStrip.apriltagDiag;
    }  
    if (m_robotContainer.isFrontLeftSwerveReady()){
      diagState += LEDStrip.swerve1Diag; 
    }
    if (m_robotContainer.isFrontRightSwerveReady()){
      diagState += LEDStrip.swerve2Diag; 
    }
    if (m_robotContainer.isBackLeftSwerveReady()){
      diagState += LEDStrip.swerve3Diag; 
    }
    if (m_robotContainer.isBackRightSwerveReady()){
      diagState += LEDStrip.swerve4Diag; 
    }
    
    /*
    ledstrip.setDiagnosticPattern(diagState);
    ledstrip.diagnosticLEDmode();
    ledstrip.setLEDMode(LEDModes.SOLIDBLUE);
    */
    // if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    // {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    // }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);

    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    SmartDashboard.putData("PID tuning choices", pidTuningChooser);
    double tunePID_Dashboard_P = SmartDashboard.getNumber("TunePID P Gain", 0);
    double tunePID_Dashboard_I = SmartDashboard.getNumber("TunePID I Gain", 0);
    double tunePID_Dashboard_D = SmartDashboard.getNumber("TunePID D Gain", 0);
    double tunePID_Dashboard_Iz = SmartDashboard.getNumber("TunePID I Zone", 0);
    double tunePID_Dashboard_FF = SmartDashboard.getNumber("TunePID Feed Forward", 0);
    double tuneShot_Dashboard_LeftRPM = SmartDashboard.getNumber("TuneShot Left RPM", 0);
    double tuneShot_Dashboard_RightRPM = SmartDashboard.getNumber("TuneShot Right RPM", 0);
    if (tunePID_Dashboard_P != tunePID_KP || tunePID_Dashboard_I != tunePID_KI || tunePID_Dashboard_D != tunePID_KD || tunePID_Dashboard_Iz != tunePID_KIz || tunePID_Dashboard_FF != tunePID_KFF || tuneShot_Dashboard_LeftRPM != tuneShot_KLeftRPM || tuneShot_Dashboard_RightRPM != tuneShot_KRightRPM) {
      if (pidTuningChooser.getSelected().equals("ShooterMotorPIDF")){
         m_robotContainer.setShooterMotorsPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      }
      else if (pidTuningChooser.getSelected().equals("ElbowPID")){
         m_robotContainer.setElbowPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      }
      else if (pidTuningChooser.getSelected().equals("ShoulderPID")){
         m_robotContainer.setShoulderPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      }
      else if (pidTuningChooser.getSelected().equals("ShooterAnglePID")){
         m_robotContainer.setShooterAnglePIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      }
      else if (pidTuningChooser.getSelected().equals("AutoAimPID")){
         m_robotContainer.setAutoAimPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      }
      else if (pidTuningChooser.getSelected().equals("ShooterSpeeds")) {
        m_robotContainer.setShooterTargets(tuneShot_Dashboard_LeftRPM, tuneShot_Dashboard_RightRPM);
      }
      else if (pidTuningChooser.getSelected().equals("WristPID")) {
        m_robotContainer.setWristPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF);
      }
      // else if (pidTuningChooser.getSelected().equals("AutoGrabNote_X_PID")){
      //    m_robotContainer.setAutoGrabNoteXPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      // }
      // else if (pidTuningChooser.getSelected().equals("AutoGrabNote_Y_PID")){
      //    m_robotContainer.setAutoGrabNoteYPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      // }
      // else if (pidTuningChooser.getSelected().equals("AutoGrabNote_R_PID")){
      //    m_robotContainer.setAutoGrabNoteRPIDF(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D, tunePID_Dashboard_Iz, tunePID_Dashboard_FF); 
      // }
      tunePID_KP = tunePID_Dashboard_P;
      tunePID_KI = tunePID_Dashboard_I;
      tunePID_KD = tunePID_Dashboard_D;
      tunePID_KIz = tunePID_Dashboard_Iz;
      tunePID_KFF = tunePID_Dashboard_FF;
    }
    
    
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}