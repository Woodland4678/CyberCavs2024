// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public        double      maximumSpeed = Units.feetToMeters(20.04); 

  private Relay headlights;
  private NetworkTable limelight;
  //private NetworkTable rpi;
  private PhotonCamera rpi;
  double autoAimP = 0;
  double autoAimI = 0;
  double autoAimD = 0;
  double autoAimIZ = 0;
  double autoAimFF = 0;

  int[] moduleCheckCount = {0,0,0,0};
  boolean[] isModuleOkay = {false, false, false, false};
  double[] prevModuleReading = {0,0,0,0};

  boolean isLimelightOkay = false;
  int checkLimeLightCount = 0;

  boolean isRPIOkay = false;
  int checkRPICount = 0;

  int gyroCheckCount = 0;
  boolean isGyroOkay = false;
  double prevGyroReading = 0;


  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.42857);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4 * 0.94), (5.90625 / 1.0)); //6.75 * 14/16  its just 6.75 for practice frame
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    headlights = new Relay(Constants.Swerve.headlightsRelayChannel);
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    rpi = new PhotonCamera("Arducam");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      //swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      swerveDrive.resetDriveEncoders();
      //swerveDrive.swerveController.addSlewRateLimiters(new SlewRateLimiter(5.5), new SlewRateLimiter(5.5), new SlewRateLimiter(3.5));
      
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    setupPathPlanner();
    
  }
  public void resetSwerveModules() {
    swerveDrive.resetDriveEncoders();
  }
  public void setHeadlights(boolean on) {
    if (!on) {
      headlights.set(Relay.Value.kOff);
    }
    else {
      headlights.set(Relay.Value.kForward);
    }
  }
  
  public void stop() {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));    
  }
  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  /**
   * Limelight Functions
   */
  public void setLimelightPipeline(int pipelineNumber) {
    limelight.getEntry("pipeline").setNumber(pipelineNumber);
  }
  public double getLimelightX() {
    return limelight.getEntry("tx").getDouble(0);
  }
  public double getLimelightObjectSize() {
    return limelight.getEntry("ta").getDouble(0);
  }
  public double getLimelightY() {
    return limelight.getEntry("ty").getDouble(0);
  }
  public double limelightHasTarget() {
    return limelight.getEntry("tv").getDouble(0);
  }
  public void setLimelightLED(boolean state) {
    if(state == false)
      limelight.getEntry("ledMode").setNumber(1);
    else {
      limelight.getEntry("ledMode").setNumber(3);
    }
  
  }
  public double getMaximumVelocity() {
    return swerveDrive.getMaximumVelocity();
  }
  public double getMaximumAngularVelocity() {
    return swerveDrive.getMaximumAngularVelocity();
  }
  public boolean isModuleReady(int module) {
    // if (moduleCheckCount[module] >= 100) {
    //   if (swerveDrive.getModulePositions()[module].angle.getDegrees() != prevModuleReading[module]) {
    //     isModuleOkay[module] = true;
    //   }
    //   else {
    //     isModuleOkay[module] = false;
    //   }
    //   prevModuleReading[module] = swerveDrive.getModulePositions()[module].angle.getDegrees();
    //   moduleCheckCount[module] = 0;
    // }
    // moduleCheckCount[module]++;
   
    // return isModuleOkay[module];

    double degrees = swerveDrive.getModules()[module].getAbsolutePosition(); 
     if (degrees != 0) {
      return true;
     }
    return false;
    //return false;
  }
  public boolean isGyroReady() {
    AHRS tGyro = (AHRS)swerveDrive.getGyro().getIMU();
    return tGyro.isConnected();
    //SmartDashboard.putNumber("TestGryoVal", this.getHeading().getDegrees());
    // if (gyroCheckCount >= 100) {
    //   if (this.getHeading().getDegrees() != prevGyroReading) {
    //     isGyroOkay = true;
    //   }
    //   else {
    //     isGyroOkay = false;
    //   }
    //   prevGyroReading = this.getHeading().getDegrees();
    //   gyroCheckCount = 0;
    // }
    // gyroCheckCount++;
    // return isGyroOkay;
    //return gyro.isConnected();
    // double curGyro = getYaw().getDegrees();
    // if (curGyro < chkGyroMinValue) {
    //   chkGyroMinValue = curGyro;
    // }
    // else if (curGyro > chkGyroMaxValue) {
    //   chkGyroMaxValue = curGyro;
    // }
    // if (Math.abs(Math.abs(chkGyroMaxValue) - Math.abs(chkGyroMaxValue)) > 0.5) {
    //   return false;
    // }
    // if (Math.abs(getYaw().getDegrees() - chkGyroCurrentValue) != 0) {
    //   chkGyroCnt = 50;
    //   return true;
    // }
    // else {
    //   chkGyroCnt--;
    // }
    // if (chkGyroCnt == 0) {
    //   chkGyroCnt = 50;
    //   return false;
    // }
    // chkGyroCurrentValue = curGyro;
    // return false;
  }
  public boolean isLimelightReady() {
    InetAddress limelightIP;
    checkLimeLightCount++;
    if (checkLimeLightCount > 100) {
      checkLimeLightCount = 0;
      try {
        limelightIP = InetAddress.getByName("10.46.78.12");
        boolean reachable = limelightIP.isReachable(3000);
        if (reachable) {
          isLimelightOkay =  true;
        }
        else {
          isLimelightOkay = false;
        }
      } catch (UnknownHostException e) {
        // TODO Auto-generated catch block
        isLimelightOkay = false;
      } catch (IOException e) {
        // TODO Auto-generated catch block
        isLimelightOkay = false;
      }
    }
    return isLimelightOkay;
  }
  public boolean isAprilTagCameraReady() {
    InetAddress rpiIP;
    checkRPICount ++;
    if (checkRPICount > 100) {
      checkRPICount = 0;
      try {
        rpiIP = InetAddress.getByName("10.46.78.11");
        boolean reachable = rpiIP.isReachable(3000);
        if (reachable) {
          isRPIOkay = true;
        }
        else {
          isRPIOkay = false;
        }
      } catch (UnknownHostException e) {
        // TODO Auto-generated catch block
        isRPIOkay = false;
      } catch (IOException e) {
        // TODO Auto-generated catch block
        isRPIOkay = false;
      }
    }
    return isRPIOkay;
    
  }
  /** photon vision code */
public double getAprilTagX() {
  Optional<Alliance> ally = DriverStation.getAlliance();
  if (rpi.getLatestResult().getBestTarget() != null) {
    var results = rpi.getLatestResult().getTargets();
    if (ally.get() == Alliance.Blue) {
      for (int i = 0; i < results.size(); i++) {
        if (results.get(i).getFiducialId() == 7) {
          return results.get(i).getYaw();
        }
      }
    }
    else {
      for (int i = 0; i < results.size(); i++) {
        if (results.get(i).getFiducialId() == 3) {
          return results.get(i).getYaw();
        }
      }
    }
   // return rpi.getLatestResult().getBestTarget().getYaw();
  }
  else {
    return 0;
  }
  return 0;
}
public boolean hasAprilTagTarget() {
  if (rpi.getLatestResult().getBestTarget() != null) {
   // if (rpi.getLatestResult().getBestTarget().getFiducialId() == 7 || (rpi.getLatestResult().getBestTarget().getFiducialId() == 3)) {
      return true;
    //}
  }
  return false;
}
public int getAprilTagTargetID() {
   if (rpi.getLatestResult().getBestTarget() != null) {
    var results = rpi.getLatestResult().getTargets();
    if (results.size() > 0) {
      return results.get(0).getFiducialId();
    }
  }
  return -1;
}
public double getAprilTagY() {
  Optional<Alliance> ally = DriverStation.getAlliance();
  if (rpi.getLatestResult().getBestTarget() != null) {
    var results = rpi.getLatestResult().getTargets();
    if (ally.get() == Alliance.Blue) {
      for (int i = 0; i < results.size(); i++) {
        if (results.get(i).getFiducialId() == 7) {
          return results.get(i).getPitch();
        }
      }
    }
    else {
      for (int i = 0; i < results.size(); i++) {
        if (results.get(i).getFiducialId() == 3) {
          return results.get(i).getPitch();
        }
      }
    }
   // return rpi.getLatestResult().getBestTarget().getYaw();
  }
  else {
    return 0;
  }
  return 0;
}

public void setAutoAimPIDF(double p, double i, double d, double iz, double f) {
  autoAimP = p;
  autoAimI = i;
  autoAimD = d;
  autoAimIZ = iz;
  autoAimFF = f;
}
public double getAutoAimP() {
  return autoAimP;
}
public double getAutoAimI() {
  return autoAimI;
}
public double getAutoAimD() {
  return autoAimD;
}
public double getAutoAimIZ() {
  return autoAimIZ;
}
public double getAutoAimFF() {
  return autoAimFF;
}

/**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                        //translation constants
                                         new PIDConstants(7, 0.0, 0.0),
                                         //rotation constants
                                         new PIDConstants(5, 0.0, 0.0),
                                         // Translation PID constants
                                        //  new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                        //                   swerveDrive.swerveController.config.headingPIDF.i,
                                        //                   swerveDrive.swerveController.config.headingPIDF.d),
                                         // Rotation PID constants
                                         5,
                                         // Max module speed, in m/s
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
  }
  public Command findPath(PathPlannerPath path, PathConstraints constraints, double rotationDelay) {
    return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                rotationDelay // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }
  public Command findPathToPose(double x, double y, double rotation, PathConstraints constraints, boolean isRedAlliance) {
    if (isRedAlliance) {
      return AutoBuilder.pathfindToPoseFlipped(new Pose2d(new Translation2d(x, y), new Rotation2d(Math.toRadians(rotation))), constraints);    
    }
    else {
      return AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(x, y), new Rotation2d(Math.toRadians(rotation))), constraints);    
    }
  }
  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  public Command followPathCommand(String pathName) {        
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getFieldVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveFieldOriented, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.1, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
  


  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String autoname, boolean setOdomToStart)
  {
    // Load the path you want to follow using its name in the GUI
   // PathPlannerPath path = PathPlannerPath.fromPathFile(autoname);

  //  if (setOdomToStart)
  //  {
  //    resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
  //  }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.buildAuto(autoname);
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity() * 0.75,
                        true,
                        false);
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic()
  {
   // this.drive(new ChassisSpeeds(0,0,1));2
    SmartDashboard.putNumber("Limelight X", getLimelightX());
    SmartDashboard.putNumber("Limelight Y", getLimelightY());
    SmartDashboard.putNumber("Limelight obj size", getLimelightObjectSize());
    SmartDashboard.putNumber("Swerve Module 0 speed",swerveDrive.getModules()[0].getDriveMotor().getAppliedOutput());
    SmartDashboard.putNumber("Swerve X Speed", swerveDrive.getFieldVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Swerve Max speed",getMaximumVelocity());
    SmartDashboard.putNumber("swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters()", swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters());
   // swerveDrive.getModules()[0].getDriveMotor().getVelocity();

  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  // public Rotation2d getHeading() {
  //   // return (Constants.Swerve.invertGyro)
  //   //     ? Rotation2d.fromDegrees(360 - gyro.getYaw())
  //   //     : Rotation2d.fromDegrees(gyro.getYaw());

  //   // if (gyro.isMagnetometerCalibrated()) {
  //   //     // We will only get valid fused headings if the magnetometer is calibrated
  //   //     return Rotation2d.fromDegrees(gyro.getFusedHeading());
  //   //     }
  //   //
  //   //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
  //       double gyroYaw = -swerveDrive.getGyroRotation3d().getZ() + 180;
  //       if (gyroYaw > 180) {
  //         gyroYaw -= 360;
  //       }
  //       else if (gyroYaw < -180) {
  //         gyroYaw += 360;
  //       }
  //       return Rotation2d.fromDegrees(gyroYaw);
  // }
  

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
  public double AutoAim() {
    // Fill later
    double error = 0; // Some number
    return error;
  }
  public void flipHeadingForRed() {
    swerveDrive.resetOdometry(new Pose2d(new Translation2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY()), new Rotation2d(Math.toRadians(swerveDrive.getOdometryHeading().getDegrees() + 180))));
  }
}