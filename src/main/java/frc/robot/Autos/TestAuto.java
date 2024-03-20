// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoGrabNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup { 
  /** Creates a new TestAuto. */
  public TestAuto(SwerveSubsystem S_Swerve, Arm S_Arm, Intake S_Intake, Command Path1, Pose2d startingPos) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> S_Swerve.resetOdometry(PathPlannerPath.fromPathFile("SideWoofer1").getPreviewStartingHolonomicPose())),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer1")),
      S_Swerve.findPath(PathPlannerPath.fromPathFile("SideWoofer2"), new PathConstraints(4, 4.5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("SideWoofer3"))
      //new InstantCommand(() -> S_Swerve.resetSwerveModules()),
     // new InstantCommand(() -> S_Swerve.setHeadlights(true)),
     // new InstantCommand(() -> S_Arm.MoveArm(Constants.ArmConstants.restPosition)),
     // new InstantCommand(() -> S_Swerve.resetOdometry(startingPos)),
      //Path1
      //new AutoGrabNote(S_Swerve, S_Intake, true),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("SideWoofer2"), new PathConstraints(2, 2, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5)
     // new AutoAim(S_Swerve, null, null, null, isScheduled())
      //S_Swerve.getAutonomousCommand("ClearSideToMiddleWoofer", true)
     // new AutoGrabNote(S_Swerve)
    );
  }
}
