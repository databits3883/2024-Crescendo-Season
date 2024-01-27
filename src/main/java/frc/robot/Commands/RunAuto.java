// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.Commands;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;

//testing github
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAuto extends InstantCommand {
  public PathPlannerAuto trajectory;
  public DriveSubsystem robotDrive;
  public FollowPathWithEvents commandWithEvents;
  public boolean stopOnFinish;
  public boolean robotRelative;

  public RunAuto(PathPlannerAuto traj, DriveSubsystem driveSubsystem, boolean robotCentric) {
    trajectory = traj;
    robotDrive = driveSubsystem;
    robotRelative = robotCentric;

    /*PPSwerveControllerCommand driveCommand = 
    new PPSwerveControllerCommand(trajectory,
     robotDrive::getPose,
    DriveConstants.kDriveKinematics, 
    DriveConstants.kXController,
    DriveConstants.kYController,
    DriveConstants.kThetaController,
     robotDrive::setModuleStates,
      robotDrive);

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      //eventMap.put("intakeDown", new IntakeDown());

  commandWithEvents = new FollowPathWithEvents(
    driveCommand,
    traj.getMarkers(),
    eventMap
);
      
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
      robotDrive.m_fieldPose.getObject("traj").setTrajectory(trajectory);
 

        
        
        commandWithEvents.schedule();

    }
}
*/