// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final int kTopIntakeMotorID = 18;
    public static final int kBottomIntakeMotorID = 21;

    public static final int kLaunchMotorLeaderID = 19;
    public static final int kLaunchMotorFollowerID = 20;

    public static final int kArmAngleMotor1ID = 14;
    public static final int kArmAngleMotor2ID = 15;
    public static final int kArmAngleMotor3iID = 16;
    public static final int kArmAngleMotor4iID = 17;


    public static final double kAngleP = 0.003;
    public static final double kAngleI = 0.00325;
    public static final double kAngleD = 0;
    public static final double kAngleVelTolerance = 0;
    public static final double kAnglePosTolerance = 0;
    public static final double kAngleIZone = Double.POSITIVE_INFINITY;


    public static final double kLaunchSpeedP = 0.002;//0.021
    public static final double kLaunchSpeedI = 0;//0
    public static final double kLaunchSpeedD = 0.0;//0.04
    public static final double kLaunchSpeedFF = 0.0035;//0.003
    public static final double kLaunchSpeedPosTolerance = 0;
    public static final double kLaunchSpeedVelTolerance = 0;
    public static final double kLaunchSpeedIZone = 0;

    public static final double kLaunchPosConversionFactor = 4*0.0254*Math.PI;
    public static final double kLaunchVelConversionFactor = 4*0.0254*Math.PI / 60;


    public static final int kFlapServoChannel = 1;
    public static final int kClimbLockServoChannel = 2;
    
    public static final double kFlapServoOpenPos = 0.4;
    public static final double kFlapServoClosedPos = 1.0;

    public static final double kIntakeP = 0.0002;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kIntakeIZone = 0;
    public static final double kIntakeFF = 0.003;

    public static final double kIntakePosConversionFactor = 1/(Units.inchesToMeters(2)*Math.PI * 50);
    public static final double kIntakeVelConversionFactor = kIntakePosConversionFactor / 60;

    public static final double kSpeakerHeight = 2.0;

    public static final double kArmPosNearStaticLaunch = 20;
    public static final double kArmPosFarStaticLaunch = 30;
    public static final double kArmPosPickup = -1;
    public static final double kArmPosAmp = 140;
    public static final double kArmPosClimbPrep = 90;
    public static final double kArmPosClimbFinish = 25;

    

    
  }

   public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double ROBOT_MAX_SPEED = Units.feetToMeters(19.3);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double FIELD_WIDTH = Units.inchesToMeters(653.2);  // (76.1 + 250.5 ) * 2 = 653.2 inches

  public static final String ROBOT_SUPERSONIC_CONFIG_LOCATION = "swerve/sonicsparkflex";

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
    public static final int kCopilotControllerPort = 1;
    public static final double kDriveStickPower = 2;
  }

  public static final class DriveConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 4.267;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  }

  public static final class VisionConstants
  {
    public static final boolean hasCamera = false;
    //Camera name in network tables
    public static final String cameraName = "Camera_Module_v1";
    //Position of the camera from center of the robot in meters
    public static final double cameraZ = Units.inchesToMeters(14.25);
    public static final double cameraX = Units.inchesToMeters(1);
    public static final double cameraY = Units.inchesToMeters(10);
    //Pipeline name in network tables
    public static final String pipelineName = "apriltag";

    //How accurate the bot needs the estimated position to be to update field pos (lower = better)
    public static final double acceptibleAmbiguity = 0.25;

    //If false, will not automatically update our pose
    public static final boolean autoUpdatePose = true;
  }

  public static final class PoseConstants
  {
    public static final Pose2d chainID15 = new Pose2d(4.45, 4.94, Rotation2d.fromDegrees(-0));
    public static final Pose2d chainID14 = new Pose2d(5.86, 4.11, Rotation2d.fromDegrees(0));
    public static final Pose2d chainID16 = new Pose2d(4.45, 3.27, Rotation2d.fromDegrees(0));

    public static final Pose2d blueOnePose = new Pose2d(0.70,6.69,Rotation2d.fromDegrees(0));
    public static final Pose2d redOnePose = new Pose2d(Constants.FIELD_WIDTH - blueOnePose.getX(), blueOnePose.getY(), Rotation2d.fromDegrees(180 - Units.rotationsToDegrees(blueOnePose.getRotation().getRotations())));
    public static final Pose2d blueTwoPose = new Pose2d(1.39,5.54,Rotation2d.fromDegrees(0));
    public static final Pose2d redTwoPose = new Pose2d(Constants.FIELD_WIDTH - blueTwoPose.getX(), blueTwoPose.getY(), Rotation2d.fromDegrees(180 - Units.rotationsToDegrees(blueTwoPose.getRotation().getRotations())));
    public static final Pose2d blueThreePose = new Pose2d(0.70,4.38,Rotation2d.fromDegrees(0));
    public static final Pose2d redThreePose = new Pose2d(Constants.FIELD_WIDTH - blueThreePose.getX(), blueThreePose.getY(), Rotation2d.fromDegrees(180 - Units.rotationsToDegrees(blueThreePose.getRotation().getRotations())));
    public static final Pose2d[] initRobotPoses = {blueOnePose, blueTwoPose, blueThreePose, redOnePose, redTwoPose, redThreePose};
  
    public static final Pose2d autoEndPose = new Pose2d(5.891426328307202, 6.045027362781998, Rotation2d.fromDegrees(90));
  }

}
