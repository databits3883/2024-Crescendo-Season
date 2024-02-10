package frc.robot.Commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class driveToPose extends Command {
    private SwerveSubsystem drivebase;
    private Pose2d pose;

    public driveToPose(SwerveSubsystem drivebase, Pose2d pose) {
        this.drivebase = drivebase;
        this.pose = pose;
    }

    @Override
    public void execute(){
        drivebase.driveToPose(pose);
    }

    @Override
    public void end(boolean interrupted) {
        
    }


    
}
