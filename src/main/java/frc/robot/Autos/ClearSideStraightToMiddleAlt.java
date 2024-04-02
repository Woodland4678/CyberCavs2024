// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.AutoAdjustShooterAngle;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoGrabNote;
import frc.robot.commands.CalibrateWrist;
import frc.robot.commands.ClimberDown;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClearSideStraightToMiddleAlt extends SequentialCommandGroup {
  PathConstraints defaultContraints = new PathConstraints(4.5, 6.5, Units.degreesToRadians(540), Units.degreesToRadians(720));
    
  /** Creates a new ClearSideToMiddleWoofer. */
  public ClearSideStraightToMiddleAlt(SwerveSubsystem S_Swerve, Intake S_Intake, Arm S_Arm, Shooter S_Shooter, Climber S_Climber, Command[] Paths,  CommandXboxController notRealController, boolean isRedAlliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Pose2d initialPose; 
    if (isRedAlliance) {
        initialPose = PathPlannerPath.fromPathFile("ClearSideStraightToMiddle1Alt").flipPath().getPreviewStartingHolonomicPose();
      }
      else {
        initialPose = PathPlannerPath.fromPathFile("ClearSideStraightToMiddle1Alt").getPreviewStartingHolonomicPose();
      }
    addCommands(
      new InstantCommand(() -> S_Swerve.resetOdometry(initialPose)),
      new InstantCommand(() -> S_Swerve.resetSwerveModules()),
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      new InstantCommand(() -> S_Arm.MoveArm(Constants.ArmConstants.restPosition)),
      new ParallelDeadlineGroup(
        Paths[0], 
        new CalibrateWrist(S_Arm), 
        new ClimberDown(S_Climber)
      ),
      new AutoGrabNote(S_Swerve, S_Intake, true, notRealController, false),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(3.95, 2.62, -40.91, defaultContraints, isRedAlliance),
       // AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(3.95, 2.62), new Rotation2d(Math.toRadians(-40.91))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)
      ),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle2"), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      Paths[1],
      new AutoGrabNote(S_Swerve, S_Intake, true, notRealController, false),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(3.49, 2.58, -42.44, defaultContraints, isRedAlliance),
        //AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(3.49, 2.58), new Rotation2d(Math.toRadians(-42.44))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)       
      ),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("ClearSideStraightToMiddle4"), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      Paths[2],
      new AutoGrabNote(S_Swerve, S_Intake, true, notRealController, false),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(2.58, 3.45, -44.5, defaultContraints, isRedAlliance),
        //AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(2.58, 3.45), new Rotation2d(Math.toRadians(-44.5))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)       
      ),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true)
    );
  }
}
