// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.TestAuto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem S_Swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
        () -> -m_driverController.getRawAxis(4));
    S_Swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // S_Swerve.setDefaultCommand(new TeleopSwerve(S_Swerve,
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> m_driverController.getRawAxis(4)));
    
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.back().onTrue(new InstantCommand(() -> S_Swerve.zeroGyro()));
    m_driverController.start().onTrue(new InstantCommand(() -> S_Swerve.resetSwerveModules()));
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
}
