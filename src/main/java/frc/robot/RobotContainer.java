// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.TestAuto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoGrabNote;
import frc.robot.commands.CalibrateWrist;
import frc.robot.commands.MoveArmAmp;
import frc.robot.commands.MoveArmToRest;
import frc.robot.commands.MoveArmTrap;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.QuickShoot;
import frc.robot.commands.ReverseNoteOutOfBot;
import frc.robot.commands.SubwooferShot;
import frc.robot.commands.NormalShoot;
import frc.robot.commands.PassNoteToArm;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// power distribution module
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem S_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final Shooter S_Shooter = new Shooter();
  private final Intake S_Intake = new Intake();
  private final Arm S_Arm = new Arm();
  private final PowerDistribution PHD = new PowerDistribution();
  private final Climber S_Climber = new Climber(PHD);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private LEDStrip ledStrip;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    ledStrip = LEDStrip.getInstance();
    configureBindings();
    // Configure the trigger bindings
    Command driveFieldOrientedDirectAngle = S_Swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());

      Command driveFieldOrientedDirectAngleSim = S_Swerve.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(4));

        Command driveFieldOrientedAnglularVelocity = S_Swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(4));
    S_Swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // S_Swerve.setDefaultCommand(new TeleopSwerve(S_Swerve,
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_driverController.getRawAxis(4)));

    NamedCommands.registerCommand("AutoGrabNote", new AutoGrabNote(S_Swerve, S_Intake));
    NamedCommands.registerCommand("QuickShoot", new QuickShoot(S_Shooter, S_Intake));
    NamedCommands.registerCommand("Shoot", new NormalShoot(S_Shooter, S_Swerve, S_Intake));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    /* Driver Right Trigger = Shoot
     * Right Bumper = Amp/Trap Scsore
     * Left Bumper = engage intake manually
     * Left Trigger = Auto Grab Note
     * Y = subwoofer shot
     * Down on D = spit out note
     * back/start = reset swerve modules / reset gyro
     */
    /*Operator 
     * A = amp arm pos
     * X = rest pos
     * Y = trap pos
     * B = send Note to arm
     * Up on D= highest climber
     * right on D = mid climb
     * down on D held = climb up
     * right Trigger = pre shoot (send note to shooter)
     * back button = calibrate rest
     * start button = reset swerve modules
     */
    m_driverController.back().onTrue(new InstantCommand(() -> S_Swerve.zeroGyro()));
    m_driverController.start().onTrue(new InstantCommand(() -> S_Swerve.resetSwerveModules()));
   // m_driverController.a().whileTrue(new AutoGrabNote(S_Swerve, S_Intake));
    //m_driverController.a().onTrue(new MoveArmAmp(S_Arm));
    //m_driverController.b().onTrue(new MoveArmToRest(S_Arm));
    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setRightAndLeftRPM(-3000,-2500)));
    m_driverController.x().onTrue(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
    m_driverController.leftTrigger().whileTrue(new AutoGrabNote(S_Swerve, S_Intake));
    m_driverController.y().whileTrue(new SubwooferShot(S_Shooter, S_Intake));
    //m_driverController.y().whileTrue(new AutoAim(S_Swerve, m_driverController));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> S_Intake.setAllMotorsPercentOutput(-0.5,0.5,-0.5, 0.5)));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> S_Intake.setAllMotorsPercentOutput(0.5,-0.5,0.5, -0.5)));
    m_driverController.pov(270).onTrue(new InstantCommand(() -> S_Intake.stopIntakeMotors()));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> S_Arm.setRollerOutputPercent(0.3)));
    m_driverController.rightBumper().onFalse(new InstantCommand(() -> S_Arm.stopArmRollers()));
    //m_driverController.leftTrigger().onTrue(new InstantCommand(() -> S_Arm.setRollerOutputPercent(-0.30)));
    //m_driverController.rightBumper().onTrue(new InstantCommand(() -> S_Arm.stopArmRollers()));
   // m_driverController.pov(0).onTrue(new PassNoteToArm(S_Arm, S_Intake));
    //m_driverController.pov(0).onTrue(new CalibrateWrist(S_Arm));
    m_driverController.pov(0).onTrue(new InstantCommand(() -> S_Shooter.setShooterAngle(92)));
    
    m_operatorController.a().onTrue(new MoveArmAmp(S_Arm));
    m_operatorController.b().onTrue(new PassNoteToArm(S_Arm, S_Intake));
    m_operatorController.y().onTrue(new MoveArmTrap(S_Arm));
    m_operatorController.x().onTrue(new MoveArmToRest(S_Arm));
    m_operatorController.back().onTrue(new CalibrateWrist(S_Arm));
    //m_operatorController.rightTrigger(new SendNoteToShooter());
   // m_operatorController.pov(0).onTrue(new ReverseNoteOutOfBot(S_Intake));
   // m_operatorController.pov(90).onTrue(new MoveClimber(S_Climber, 0)); //Change Climber Position
   // m_operatorController.leftBumper().onTrue(new MoveArmToRest(S_Arm));
  //  m_operatorController.rightBumper().onTrue(new MoveArmAmp(S_Arm));
  }

  /**`
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new TestAuto(S_Swerve);
  }
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    S_Swerve.setMotorBrake(brake);
  }
public void setElbowPIDF(double p, double i, double f, double iz, double ff) {
    S_Arm.setElbowPIDF(p, i, f, iz, ff);
  }
  public void setShoulderPIDF(double p, double i, double f, double iz, double ff) {
    S_Arm.setShoulderPIDF(p, i, f, iz, ff);
  }

  public void setShooterAnglePIDF(double p, double i, double f, double iz, double ff) {
    S_Shooter.setShooterAnglePIDF(p, i, f, iz, ff);
  }
  public void setShooterMotorsPIDF(double p, double i, double d, double iz, double ff) {
    S_Shooter.setShooterMotorsPIDF(p, i, d, iz, ff);
  }
  public void setAutoAimPIDF(double p, double i, double d, double iz, double ff) {
    S_Swerve.setAutoAimPIDF(p, i, d, iz, ff);
  }
  public void resetArmAngles() {
    S_Arm.resetToAbsolute();
  }
  public void turnOffClimberLock() {
    S_Climber.disengageLock();
  }
  public void engageClimberLock() {
    S_Climber.engageLock();
  }
 public boolean isShoulderReady () {
    return S_Arm.isShoulderReady();
 }
 public boolean isElbowReady(){
    return S_Arm.isElbowReady();
 }  
 public boolean isFrontLeftSwerveReady(){
  return S_Swerve.isModuleReady(0);
 }
 public boolean isFrontRightSwerveReady(){
  return S_Swerve.isModuleReady(1);
 }
 public boolean isBackLeftSwerveReady(){
  return S_Swerve.isModuleReady(2);
 }
 public boolean isBackRightSwerveReady(){
  return S_Swerve.isModuleReady(3);
 }
 public boolean isGyroReady(){
  return S_Swerve.isGyroReady();
 }
 public boolean isLimelightReady(){
  return true;
 }
 public boolean isAprilTagCameraReady(){
  return false;
 }
}