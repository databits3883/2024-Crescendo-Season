// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ScoringArmConstants {
    public static final int kIntakeMotorID = 10;
    public static final int kLaunchMotorLeaderID = 11;
    public static final int kLaunchMotorFollowerID = 12;
    public static final int kArmAngleMotorLeader = 13;
    //public static final int kArmAngleMotorFollower = 29;


    public static final double kAngleP = 0;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleVelTolerance = 0;
    public static final double kAnglePosTolerance = 0;
    public static final double kAngleIZone = 0;


    public static final double kLaunchSpeedP = 0;
    public static final double kLaunchSpeedI = 0;
    public static final double kLaunchSpeedD = 0;
    public static final double kLaunchSpeedIZone = 0;
    public static final double kLaunchSpeedPosTolerance = 0;
    public static final double kLaunchSpeedVelTolerance = 0;
    public static int kFlapServo1Channel=0;
    public static int kFlapServo2Channel=1;

  }

  public static final class ClimbConstants{

    public static final int kLeftArmMotorChannel = 0;
    public static final int kRightArmMotorChannel = 0;

  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;  
    public static final double kDriveStickPower = 2;
    public static int kCopilotControllerPort = 1;
  }

  public static final class DriveConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 4.267;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  }

  public static final class VisionConstants
  {
    //Position of the camera from center of the robot in meters
    public static final double cameraZ = Units.inchesToMeters(12.5);
    public static final double cameraX = Units.inchesToMeters(1);
    public static final double cameraY = Units.inchesToMeters(10);
  }
}
