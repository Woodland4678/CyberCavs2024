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
public class AmpSideStraightToMiddle extends SequentialCommandGroup {
  PathConstraints defaultContraints = new PathConstraints(4.1, 6, Units.degreesToRadians(540), Units.degreesToRadians(720));
    
  /** Creates a new ClearSideToMiddleWoofer. */
  public AmpSideStraightToMiddle(SwerveSubsystem S_Swerve, Intake S_Intake, Arm S_Arm, Shooter S_Shooter, Climber S_Climber, Command[] Paths,  CommandXboxController notRealController, boolean isRedAlliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Pose2d initialPose; 
    if (isRedAlliance) {
        initialPose = PathPlannerPath.fromPathFile("AmpSideStraightToMiddle1").flipPath().getPreviewStartingHolonomicPose();
      }
      else {
        initialPose = PathPlannerPath.fromPathFile("AmpSideStraightToMiddle1").getPreviewStartingHolonomicPose();
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
     // Paths[0],
      new AutoGrabNote(S_Swerve, S_Intake, true,notRealController, true),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(4.6, 6.35, 14.04, defaultContraints, isRedAlliance),
        //AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(4.6, 6.35), new Rotation2d(Math.toRadians(14.04))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)
      ),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle2"), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true),
      Paths[1],
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      new AutoGrabNote(S_Swerve, S_Intake, true, notRealController, true),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(4.21, 6.22, 11.73, defaultContraints, isRedAlliance),
       // AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(4.21, 6.22), new Rotation2d(Math.toRadians(11.73))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)        
      ),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle4"), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true),
      Paths[2],
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      new AutoGrabNote(S_Swerve, S_Intake, true, notRealController, true),
      new ParallelRaceGroup(
        S_Swerve.findPathToPose(4.42, 4.89, -13.76, defaultContraints, isRedAlliance),
       // AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(4.42, 4.89), new Rotation2d(Math.toRadians(-13.76))), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720))),
        new AutoAdjustShooterAngle(S_Shooter, S_Intake, S_Swerve)        
      ),
      //S_Swerve.findPath(PathPlannerPath.fromPathFile("AmpSideStraightToMiddle6"), new PathConstraints(4, 5, Units.degreesToRadians(540), Units.degreesToRadians(720)), 0.5),
      new AutoAim(S_Swerve, S_Shooter, S_Intake, notRealController, true)
    );
  }
}