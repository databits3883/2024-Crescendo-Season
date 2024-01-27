// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends InstantCommand {
  public Trajectory trajectory;
  public DriveSubsystem robotDrive;
  public SwerveControllerCommand driveCommand;
  public boolean stopOnFinish;
  public boolean robotRelative;

  public FollowTrajectory(Trajectory traj, DriveSubsystem driveSubsystem, boolean stopOnCompletion, boolean robotCentric) {
    trajectory = traj;
    robotDrive = driveSubsystem;
    stopOnFinish = stopOnCompletion;
    robotRelative = robotCentric;

    // driveCommand = 
    // new SwerveControllerCommand(trajectory,
    //  robotDrive::getPose,
    // DriveConstants.kDriveKinematics, 
    // DriveConstants.kSwerveDriveController,
    //  robotDrive::setModuleStates,
    //   robotDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      if ( robotRelative){

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(trajectory.getInitialPose());
      }
 

        Command commandToRun = driveCommand;
        if(stopOnFinish){
          commandToRun = commandToRun.andThen(() -> robotDrive.drive(0, 0, 0, false));
        }
        commandToRun.schedule();

    }
}
