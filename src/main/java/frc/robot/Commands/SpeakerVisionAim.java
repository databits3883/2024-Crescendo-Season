// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FieldDriverStick;
import frc.robot.subsystems.ScoringArm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveController;

public class SpeakerVisionAim extends Command {

  public SwerveSubsystem drivetrain;
  public VisionSubsystem visionSubsystem;
  public PhotonTrackedTarget target;
  public double degreesOff = 0;
  public double initialRobotHeading = 0;
  public double pidOutput = 0;
  public PIDController angleController = new PIDController(0.001, 0, 0);
  public SwerveController swerveController;
  public FieldDriverStick driverJoystick;
  /** Creates a new SpeakerVisionAim. */
  public SpeakerVisionAim(SwerveSubsystem drive, VisionSubsystem vision, FieldDriverStick joystick) {
    drivetrain = drive;
    visionSubsystem = vision;
    driverJoystick = joystick;
    swerveController = drivetrain.getSwerveController();
    angleController.enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = visionSubsystem.getVisibleSpeakerTarget();
    if(target != null){
      degreesOff = target.getYaw() + VisionConstants.SScameraZYaw;
      System.out.println("raw yaw "+ target.getYaw());
      System.out.println("constant offset: "+ VisionConstants.SScameraZYaw);
      initialRobotHeading = drivetrain.getHeading().getDegrees();
      System.out.println("initial robot heading: "+ initialRobotHeading);
      
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double headingDeg = drivetrain.getHeading().getDegrees();
    pidOutput = angleController.calculate(headingDeg, initialRobotHeading - degreesOff) * 100;
    pidOutput += Math.signum(pidOutput) * 10;

    System.out.println("pid output: " + pidOutput);
    System.out.println("heading: "+ headingDeg);

    ChassisSpeeds speeds = new ChassisSpeeds(driverJoystick.getX(), driverJoystick.getY(), Rotation2d.fromRadians(pidOutput).plus(Rotation2d.fromDegrees(headingDeg + VisionConstants.SScameraZYaw)).getRadians());
    //drivetrain.driveFieldOriented(drivetrain.getTargetSpeeds(0.0, 0.0, Rotation2d.fromRadians( pidOutput ).plus(Rotation2d.fromDegrees(headingDeg))));
    drivetrain.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),speeds.omegaRadiansPerSecond,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(degreesOff) < 1 || target == null;
  }
}
