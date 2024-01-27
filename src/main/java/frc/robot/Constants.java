// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
    public static final int kIntakeMotorID = 25;
    public static final int kLaunchMotorLeaderID = 26;
    public static final int kLaunchMotorFollowerID = 27;
    public static final int kArmAngleMotorLeader = 28;
    public static final int kArmAngleMotorFollower = 29;


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

  }

  public static final class ClimbConstants{

    public static final int kLeftArmMotorChannel = 0;
    public static final int kRightArmMotorChannel = 0;

  }

  public static final class DriveConstants {
    public static final int kGyroPort = 14;
    public static final double kGyroFront = 0;

    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 8;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftCANCoderPort = 10;
    public static final int kRearLeftCANCoderPort = 11;
    public static final int kFrontRightCANCoderPort = 9;
    public static final int kRearRightCANCoderPort = 12;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.508;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.6508;
    // Distance between front and back wheels on robot

    public static final double kMaxSpeedMetersPerSecond = 4.267;
    public static final double kRobotDriveRadius = 1;//idk what this means, it probably needs to change
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDController kXController = new PIDController(0, 0, 0);
    public static final PIDController kYController = new PIDController(0, 0, 0);
    public static final PIDController kThetaController =
        new PIDController(
            0.47, 0, 0);

        
            //0.47
    //thetaController.enableContinuousInput(-180, 180);

    //public static final HolonomicDriveController kSwerveDriveController = new HolonomicDriveController(kXController, kYController, kThetaController);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    //public static final double ksVolts = 1;
    //public static final double kvVoltSecondsPerMeter = 0.8;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1;
    public static final double kDriveGearRatio = 1/6.75;
    public static final double kTurningGearRatio = 1/12.8;
    public static final double kWheelDiameterMeters = 4*2.54*0.01;//0.1 for black tread, and 0.0985 for blue worn tread
    public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameterMeters;
    public static final double kDriveConversionFactor =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelCircumferenceMeters * (kDriveGearRatio));
        

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (kTurningGearRatio);

    public static final double kPModuleTurningController = 1.2;
    public static final double kIModuleTurningController = 0.0;
    public static final double kDModuleTurningController = 0.0;

    public static final double kPModuleDriveController = 0.22;
    public static final double kIModuleDriveController = 0.0;
    public static final double kDModuleDriveController = 1.2;
    public static final double kFFModuleDriveController = 0.23;
    
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveStickPower = 2;
    public static int kCopilotControllerPort;
  }

  public static final class AutoConstants {


    //public static final double kPThetaController = 0.1;

    public static final TrajectoryConfig kMaxSpeedTrajectoryConfig = new TrajectoryConfig(
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);
  }

}
