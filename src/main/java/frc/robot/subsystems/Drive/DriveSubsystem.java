// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftCANCoderPort);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftCANCoderPort);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightCANCoderPort);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightCANCoderPort);

  // The gyro sensor
  //private final AHRS m_gyro = new AHRS(Port.kMXP);
  private final Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.kGyroPort);

  
  public final AutoBuilder autoBuilder = new AutoBuilder();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          getHeading(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

    public Field2d m_fieldPose = 
        new Field2d();

    public ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds(0,0,0);

    public RobotContainer rbContainer;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(RobotContainer robot) {
    calibrate();
    Shuffleboard.getTab("Debug").add("Odometry(Y is inverted)", m_fieldPose);
    Shuffleboard.getTab("Debug").addDouble("Gyro Yaw", this::getHeadingDegrees);
    rbContainer = robot;
    
    
    
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_fieldPose.setRobotPose(getPose());
    m_odometry.update(
        getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setDisplayTrajectory(Trajectory trajectory){
    m_fieldPose.getObject("traj").setTrajectory(trajectory);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      getHeading(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    currentChassisSpeeds = fieldRelative ? 
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())//is field relative
            : new ChassisSpeeds(xSpeed, ySpeed, rot);//is not field relative


    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(currentChassisSpeeds);

    setModuleStates(swerveModuleStates);
  }

  public void driveFromChassisSpeeds(ChassisSpeeds speeds){
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public void driveReversed(double xSpeed, double ySpeed,Double rot, boolean fieldRelative){
    drive(-xSpeed, -ySpeed, -rot, fieldRelative);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
    desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void calibrate(){
    m_frontLeft.calibrateTurnEncoders();
    m_rearLeft.calibrateTurnEncoders();
    m_frontRight.calibrateTurnEncoders();
    m_rearRight.calibrateTurnEncoders();

    m_gyro.setYaw(DriveConstants.kGyroFront);
    //m_gyro.setYaw(DriveConstants.kGyroFront);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.setYaw(DriveConstants.kGyroFront);
    m_gyro.setYaw(DriveConstants.kGyroFront);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
  }

  public double getHeadingDegrees(){
    return getHeading().getDegrees();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return currentChassisSpeeds;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  //public double getTurnRate() {
  //  return m_gyro.get() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  //}
}
