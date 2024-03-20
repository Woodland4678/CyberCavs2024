// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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
public class AutoBase extends SequentialCommandGroup {
  PathConstraints defaultContraints = new PathConstraints(4.1, 6, Units.degreesToRadians(540), Units.degreesToRadians(720));
    
  /** Creates a new ClearSideToMiddleWoofer. */
  public AutoBase(SwerveSubsystem S_Swerve, Intake S_Intake, Arm S_Arm, Shooter S_Shooter, Climber S_Climber, Command[] Paths, Pose2d initialPose,  CommandXboxController notRealController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> S_Swerve.resetOdometry(initialPose)),
      new InstantCommand(() -> S_Swerve.resetSwerveModules()),
      new InstantCommand(() -> S_Swerve.setHeadlights(true)),
      new InstantCommand(() -> S_Arm.MoveArm(Constants.ArmConstants.restPosition)),
      new ParallelDeadlineGroup(
        new CalibrateWrist(S_Arm), 
        new ClimberDown(S_Climber)
      )
    );
  }
}
