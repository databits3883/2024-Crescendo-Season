// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunPath extends InstantCommand {
  public PathPlannerTrajectory trajectory;
  public DriveSubsystem robotDrive;
  public PPSwerveControllerCommand driveCommand;
  public boolean stopOnFinish;
  public boolean robotRelative;

  public RunPath(PathPlannerTrajectory traj, DriveSubsystem driveSubsystem, boolean robotCentric) {
    trajectory = traj;
    robotDrive = driveSubsystem;
    robotRelative = robotCentric;

    driveCommand = 
    new PPSwerveControllerCommand(trajectory,
     robotDrive::getPose,
    DriveConstants.kDriveKinematics, 
    DriveConstants.kXController,
    DriveConstants.kYController,
    DriveConstants.kThetaController,
     robotDrive::setModuleStates,
      robotDrive);

      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      if (robotRelative){

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(trajectory.getInitialHolonomicPose());
      }
 

        
        
        driveCommand.schedule();

    }
}
*/