// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.AmpSideStraightToMiddle;
import frc.robot.Autos.AmpSideToMiddleNoWoofer;
import frc.robot.Autos.AmpSideToMiddleWoofer;
import frc.robot.Autos.AutoBase;
import frc.robot.Autos.ClearSideStraightToMiddle;
import frc.robot.Autos.ClearSideToMiddleNoCloseNote;
import frc.robot.Autos.ClearSideToMiddleWoofer;
import frc.robot.Autos.TestAuto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAdjustShooterAngle;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoGrabNote;
import frc.robot.commands.AutoGrabNote_NoR;
import frc.robot.commands.CalibrateWrist;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.FieldPass;
import frc.robot.commands.MoveArmAmp;
import frc.robot.commands.MoveArmStuckNote;
import frc.robot.commands.MoveArmToRest;
import frc.robot.commands.MoveArmTrap;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.MoveNoteForTrap;
import frc.robot.commands.QuickShoot;
import frc.robot.commands.QuickShootTwo;
import frc.robot.commands.ReverseNoteOutOfBot;
import frc.robot.commands.RotateToAmp;
import frc.robot.commands.RotateToStage;
import frc.robot.commands.SpitNote;
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
import java.nio.file.Path;
import java.util.Optional;

import org.json.simple.JSONObject;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
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
  private static final Joystick autoBox = new Joystick(2);
  private LEDStrip ledStrip;

  Command auto;
  Command ClearSideWooferPath1;
  Command[] ClearSideWooferPaths = new Command[3];
  Command[] ClearSideStraightToMiddlePaths = new Command[4];
  Command[] ClearSideStraightToMiddleAltPaths = new Command[4];
  Command[] AmpSideToMiddleWoofer = new Command[5];
  Command[] AmpSideToMiddleNoWoofer = new Command[3];
  Command[] AmpSideStraightToMiddle = new Command[3];
  Command[] AmpSideToMiddleNoWooferAlt = new Command[3];

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
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4));
    S_Swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    S_Shooter.setDefaultCommand(new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve));
    
    // S_Swerve.setDefaultCommand(new TeleopSwerve(S_Swerve,
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_driverController.getRawAxis(4)));

    NamedCommands.registerCommand("AutoGrabNote_NoR", new AutoGrabNote_NoR(S_Swerve, S_Intake, true));
    NamedCommands.registerCommand("AutoGrabNote", new AutoGrabNote(S_Swerve, S_Intake, true, m_driverController, true));
    NamedCommands.registerCommand("QuickShoot", new QuickShoot(S_Shooter, S_Intake));
    NamedCommands.registerCommand("Shoot", new AutoAim(S_Swerve, S_Shooter, S_Intake, m_driverController, true));
    NamedCommands.registerCommand("SetShooterCloseShot", new InstantCommand(() -> S_Shooter.setRightAndLeftRPM(-3000, -2600)));
    NamedCommands.registerCommand("TurnOnHeadlights", new InstantCommand(() ->S_Swerve.setHeadlights(true)));
    NamedCommands.registerCommand("SetShooterAngleCloseShot", new InstantCommand(() -> S_Shooter.setShooterAngle(77)));
    NamedCommands.registerCommand("SubwooferShot", new SubwooferShot(S_Shooter, S_Intake, S_Swerve, true));
    NamedCommands.registerCommand("SpinUpShooter", new InstantCommand(() -> S_Shooter.setRightAndLeftRPM(-4400, -3400)));
    NamedCommands.registerCommand("SetArmToRest", new InstantCommand(() -> S_Arm.MoveArm(Constants.ArmConstants.restPosition)));
     NamedCommands.registerCommand("MoveShooterTo89", new InstantCommand(() -> S_Shooter.setShooterAngle(89)));
     NamedCommands.registerCommand("QuickShootTwo", new QuickShootTwo(S_Shooter, S_Intake, S_Swerve));
    //auto = AutoBuilder.buildAuto("ClearSideStraightToMiddle");
   // ClearSideWooferPath1 = AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer1"));
   // ClearSideWooferPaths[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer1"));
    ClearSideWooferPaths[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer3"));
    ClearSideWooferPaths[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideToMiddleNoCloseNote_2"));
    
    ClearSideWooferPaths[0] = AutoBuilder.buildAuto("SideWoofer1");

    ClearSideStraightToMiddlePaths[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle1"));
    ClearSideStraightToMiddlePaths[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle3"));
    ClearSideStraightToMiddlePaths[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle5"));
    ClearSideStraightToMiddlePaths[3] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle6"));

    ClearSideStraightToMiddleAltPaths[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle1Alt"));
    ClearSideStraightToMiddleAltPaths[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle3Alt"));
    ClearSideStraightToMiddleAltPaths[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle5"));
    ClearSideStraightToMiddleAltPaths[3] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle6"));

    AmpSideToMiddleWoofer[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleWoofer1"));
    AmpSideToMiddleWoofer[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleWoofer3"));
    AmpSideToMiddleWoofer[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleWoofer5"));
    // AmpSideToMiddleWoofer[3] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer1")); 
    // AmpSideToMiddleWoofer[4] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle1"));
    
    
    AmpSideToMiddleNoWoofer[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer1"));
    AmpSideToMiddleNoWoofer[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer4"));
    AmpSideToMiddleNoWoofer[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer6"));

    AmpSideToMiddleNoWooferAlt[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer1_alt"));
    AmpSideToMiddleNoWooferAlt[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer6"));
    AmpSideToMiddleNoWooferAlt[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer2_Alt"));

    AmpSideStraightToMiddle[0] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle1"));
    AmpSideStraightToMiddle[1] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle3"));
    AmpSideStraightToMiddle[2] = AutoBuilder.followPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle5"));
      //AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer3")),
    };

  

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
    m_driverController.start().onTrue(new InstantCommand(() -> S_Swerve.resetOdometry(new Pose2d())));
    m_driverController.a().whileTrue(new RotateToAmp(S_Swerve, m_driverController));
    m_driverController.x().whileTrue(new RotateToStage(S_Swerve, m_driverController));
    
    //m_driverController.a().whileTrue(new QuickShoot(S_Shooter, S_Intake));
    //m_driverController.a().onTrue(new MoveArmAmp(S_Arm));
    //m_driverController.b().onTrue(new MoveArmToRest(S_Arm));
    //m_driverController.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setRightAndLeftRPM(-3000,-2500)));
    //m_driverController.x().onTrue(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
    // m_driverController.rightTrigger().whileTrue(new AutoAim(S_Swerve, S_Shooter, S_Intake, m_driverController, false));
    // m_driverController.leftTrigger().whileTrue(new AutoGrabNote(S_Swerve, S_Intake, false));
    m_driverController.y().whileTrue(new SubwooferShot(S_Shooter, S_Intake, S_Swerve, false));
    //m_driverController.y().whileTrue(new AutoAim(S_Swerve, m_driverController));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> S_Intake.setAllMotorsPercentOutput(-0.5,0.5,-0.5, 0.5)));
    m_driverController.leftBumper().onFalse(new InstantCommand(() ->S_Intake.stopIntakeMotors()));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> S_Intake.setAllMotorsPercentOutput(0.5,-0.5,0.5, -0.5)));
    m_driverController.pov(270).onTrue(new InstantCommand(() -> S_Intake.stopIntakeMotors()));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> S_Arm.setRollerOutputPercent(-1)));
    m_driverController.rightBumper().onFalse(new InstantCommand(() -> S_Arm.stopArmRollers()));
    m_driverController.pov(90).onTrue(new InstantCommand(() -> S_Arm.setRollerOutputPercent(0.2)));
    m_driverController.pov(90).onFalse(new InstantCommand(() -> S_Arm.stopArmRollers()));
    m_driverController.leftTrigger().whileTrue(new AutoGrabNote(S_Swerve, S_Intake, false, m_driverController, true));
    m_driverController.rightTrigger().whileTrue(new AutoAim(S_Swerve, S_Shooter, S_Intake, m_driverController, false));
    m_driverController.b().whileTrue(new SpitNote(S_Shooter, S_Intake));
    //m_driverController.rightBumper().onTrue(new InstantCommand(() -> S_Arm.stopArmRollers()));
   // m_driverController.pov(0).onTrue(new PassNoteToArm(S_Arm, S_Intake));
    //m_driverController.pov(0).onTrue(new CalibrateWrist(S_Arm));
   // m_driverController.pov(0).onTrue(new InstantCommand(() -> S_Shooter.setShooterAngle(80)));
   // m_driverController.pov(90).onTrue(new InstantCommand(() -> S_Shooter.setShooterAngle(69)));
    
    m_operatorController.a().onTrue(new MoveArmAmp(S_Arm, S_Shooter));
    m_operatorController.b().onTrue(new PassNoteToArm(S_Arm, S_Intake, S_Shooter));
    m_operatorController.y().onTrue(new MoveArmTrap(S_Arm, S_Shooter, m_operatorController));
    m_operatorController.x().onTrue(new MoveArmToRest(S_Arm, S_Shooter));
    m_operatorController.back().onTrue(new CalibrateWrist(S_Arm));
    m_operatorController.start().onTrue(new MoveArmStuckNote(S_Arm));

    //m_operatorController.pov(0).onTrue(new InstantCommand(() -> S_Shooter.increaseShooterAngle()));
    m_operatorController.pov(0).onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(Constants.ClimberConstants.maxClimberHeight)));
    m_operatorController.pov(0).onTrue(new MoveNoteForTrap(S_Arm));
  // m_operatorController.pov(0).onTrue(new InstantCommand(() -> S_Shooter.setShooterAngle(85)));
  // m_operatorController.pov(180).onTrue(new InstantCommand(() -> S_Shooter.setShooterAngle(70)));
    m_operatorController.pov(90).whileTrue(new ClimbDown(S_Climber));

    //m_operatorController.pov(180).onTrue(new InstantCommand(() -> S_Shooter.decreaseShooterAngle()));
    //m_operatorController.pov(90).onTrue(new InstantCommand(() -> S_Shooter.stopAngleMotors()));
    m_operatorController.pov(270).onTrue(new ClimberDown(S_Climber));
    //m_operatorController.leftTrigger().onTrue(new InstantCommand(() -> S_Shooter.setRightAndLeftRPM(-4000,-5000)));
    //m_operatorController.pov(270).onTrue(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
    m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> S_Intake.setAllMotorsPercentOutput(-0.5,0.5,-0.5, 0.5)));
    m_operatorController.rightTrigger().onFalse(new InstantCommand(() ->S_Intake.stopIntakeMotors()));
    m_operatorController.leftTrigger().whileTrue(new FieldPass(S_Swerve, S_Shooter, S_Intake, m_driverController));
    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> S_Climber.disengageLock()));
    m_operatorController.rightBumper().onTrue(new InstantCommand(() -> S_Climber.engageLock()));
    //m_operatorController.rightTrigger().whileTrue(new NormalShoot(S_Shooter, S_Swerve, S_Intake, 4400, 90.2, -5)); // Far stage shot
    //m_operatorController.leftTrigger().whileTrue(new NormalShoot(S_Shooter, S_Swerve, S_Intake, 4400, 81.5, -35)); // podium shot
    //m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(-79)));
    //m_operatorController.leftTrigger().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(-45)));
    m_operatorController.pov(180).whileTrue(new Climb(S_Climber));
    m_operatorController.leftStick().onTrue(new InstantCommand(() -> S_Shooter.higherShooterCalcAdjustment()));
    m_operatorController.rightStick().onTrue(new InstantCommand(() -> S_Shooter.lowerShooterCalcAdjustment()));
    m_operatorController.pov(180).onFalse(new InstantCommand(() -> S_Climber.stopClimber()));
   // m_operatorController.rightTrigger().onTrue(new InstantCommand(() -> S_Arm.setWristPosition(0)));
    // m_operatorController.leftTrigger().onTrue(new InstantCommand(() -> S_Arm.setWristPosition(45)));
    //m_operatorController.rightTrigger(new SendNoteToShooter());
   // m_operatorController.pov(0).onTrue(new ReverseNoteOutOfBot(S_Intake));
   // m_operatorController.pov(90).onTrue(new MoveClimber(S_Climber, 0)); //Change Climber Position
   // m_operatorController.leftBumper().onTrue(new MoveArmToRest(S_Arm));
  //  m_operatorController.rightBumper().onTrue(new MoveArmAmp(S_Arm));
    
 // autoBox.button(11,new InstantCommand(() -> S_Shooter.resetShooterAngle()));
  //autoBox.button(11, )
  //autoBox.getRawButton(11).onTrue(new InstantCommand(() -> S_Shooter.resetShooterAngle()));
  }

  /**`
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //return new TestAuto(S_Swerve, S_Arm, S_Intake, ClearSideWooferPath1, PathPlannerPath.fromPathFile("SideWoofer1").getPreviewStartingHolonomicPose());
   //return new ClearSideToMiddleWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, ClearSideWooferPaths, PathPlannerPath.fromPathFile("SideWoofer1").getPreviewStartingHolonomicPose(), m_driverController);
    //return new ClearSideToMiddleNoCloseNote(S_Swerve, S_Intake, S_Arm, S_Shooter, ClearSideWooferPaths, PathPlannerPath.fromPathFile("SideWoofer1").getPreviewStartingHolonomicPose(), m_driverController);
    
    //return new ClearSideStraightToMiddle(S_Swerve, S_Intake, S_Arm, S_Shooter, ClearSideStraightToMiddlePaths, PathPlannerPath.fromPathFile("ClearSideStraightToMiddle1").getPreviewStartingHolonomicPose(), m_driverController);
    //return new frc.robot.Autos.AmpSideToMiddleWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, AmpSideToMiddlePathers, PathPlannerPath.fromPathFile("AmpSideToMiddle1").getPreviewStartingHolonomicPose(), m_driverController);
   // return new AmpSideToMiddleNoWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, AmpSideToMiddleNoWoofer, PathPlannerPath.fromPathFile("AmpSideToMiddleNoWoofer1").getPreviewStartingHolonomicPose(), m_driverController);
    //   return auto;
    int autoBoxSwitch1 = 0;
    int autoBoxSwitch2 = 0;
    if (autoBox.getRawButton(1)) {
      autoBoxSwitch1 += 1;
    }
    if (autoBox.getRawButton(2)) {
      autoBoxSwitch1 += 2;
    }
    if (autoBox.getRawButton(3)) {
      autoBoxSwitch1 += 4;
    }
    if (autoBoxSwitch1 == 0) {
      autoBoxSwitch1 = 6;
    }
    else if (autoBoxSwitch1 > 5) {
      autoBoxSwitch1 = autoBoxSwitch1 + 1;
    }
    

    if (autoBox.getRawButton(8)) {
      autoBoxSwitch2 += 1;
    }
    if (autoBox.getRawButton(9)) {
      autoBoxSwitch2 += 2;
    }
    if (autoBox.getRawButton(10)) {
      autoBoxSwitch2 += 4;
    }
    if (autoBoxSwitch2 == 0) {
      autoBoxSwitch2 = 6;
    }
    else if (autoBoxSwitch2 > 5) {
      autoBoxSwitch2 = autoBoxSwitch2 + 1;
    }
    Optional<Alliance> ally = DriverStation.getAlliance();
    boolean isRedAlliance = false;
    if (ally.get() == Alliance.Red) {
      isRedAlliance = true;
    }
    if (autoBoxSwitch1 == 5 && autoBoxSwitch2 == 1) {
      return new AmpSideToMiddleNoWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, AmpSideToMiddleNoWoofer, m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 5 && autoBoxSwitch2 == 2) {
      return new frc.robot.Autos.AmpSideToMiddleNoWooferAlt(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, AmpSideToMiddleNoWooferAlt, m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 4 && autoBoxSwitch2 == 1) {
      return new ClearSideToMiddleWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, ClearSideWooferPaths, m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 4 && autoBoxSwitch2 ==2 ) {
      return new ClearSideToMiddleNoCloseNote(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, ClearSideWooferPaths, m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 3 && autoBoxSwitch2 == 2) {
      return new ClearSideStraightToMiddle(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, ClearSideStraightToMiddlePaths,  m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 3 && autoBoxSwitch2 == 1) {
      return new AmpSideStraightToMiddle(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, AmpSideStraightToMiddle, m_driverController, isRedAlliance);
    }
    else if (autoBoxSwitch1 == 4 && autoBoxSwitch2 == 3) {
      return new AmpSideToMiddleWoofer(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, AmpSideToMiddleWoofer, m_driverController, isRedAlliance);   
    }
    else {
      return new AutoBase(S_Swerve, S_Intake, S_Arm, S_Shooter, S_Climber, AmpSideToMiddleNoWoofer, new Pose2d(), m_driverController);
    }
  }
  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
  //  ??? S_Swerve.setMotorBrake(brake);
  }
public void setElbowPIDF(double p, double i, double f, double iz, double ff) {
    S_Arm.setElbowPIDF(p, i, f, iz, ff);
  }
  public void setShoulderPIDF(double p, double i, double f, double iz, double ff) {
    S_Arm.setShoulderPIDF(p, i, f, iz, ff);
  }
  public void setWristPIDF(double p, double i, double f, double iz, double ff) {
    S_Arm.setWristPIDF(p, i, f, iz, ff);
  }
  public void setClimberPIDF(double p, double i, double f, double iz, double ff) {
    S_Climber.setClimberPIDF(p, i, f, iz, ff);
  }
  public void setShooterAnglePIDF(double p, double i, double f, double iz, double ff) {
    S_Shooter.setShooterAnglePIDF(p, i, f, iz, ff);
  }
  public void setShooterMotorsPIDF(double p, double i, double d, double iz, double ff) {
    S_Shooter.setShooterMotorsPIDF(p, i, d, iz, ff);
  }
  public void setAutoAimPIDF(double p, double i, double d, double iz, double ff) {
    // ??? S_Swerve.setAutoAimPIDF(p, i, d, iz, ff);
  }
  public void resetArmAngles() {
    S_Arm.resetToAbsolute();
  }
  public void resetShooterAngle() {
    S_Shooter.resetShooterAngle();
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
  //return true;
 }
 public boolean isFrontRightSwerveReady(){
  return S_Swerve.isModuleReady(1);
  //return false;
 }
 public boolean isBackLeftSwerveReady(){
  return S_Swerve.isModuleReady(2);
  //return false;
 }
 public boolean isBackRightSwerveReady(){
  return S_Swerve.isModuleReady(3);
  //return false;
 }
 public boolean isGyroReady(){
  return S_Swerve.isGyroReady();
  //return false;
 }
 public boolean isLimelightReady(){
  return S_Swerve.isLimelightReady();
  //return true;
 }
 public boolean isAprilTagCameraReady(){
  return S_Swerve.isAprilTagCameraReady();
 }
  public void setShooterTargets(double leftRPM, double rightRPM) {
    if (leftRPM == 0 || rightRPM == 0) {
      S_Shooter.stopShooterMotor();
    } else {
      S_Shooter.setRightAndLeftRPM(rightRPM, leftRPM);
    }
  }
 public double getPDHCurrentDraw(int channel) {
  return PHD.getCurrent(channel);
 } 
 public void flipRobotHeading() {
  S_Swerve.flipHeadingForRed();
 }
}