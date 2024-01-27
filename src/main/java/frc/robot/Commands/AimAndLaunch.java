// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringArm;

public class AimAndLaunch extends Command {

  public double angleFromHorizonToSpeakerDeg = 0;
  public double distToSpeakerMeters = 0;
  public ScoringArm scoringArm;
  /** Creates a new AimAndLaunch. */
  public AimAndLaunch(ScoringArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoringArm.SetLaunchSpeed(distToSpeakerMeters);
    scoringArm.SetArmAngle(180 -(90 + angleFromHorizonToSpeakerDeg) );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted && scoringArm.anglePIDController.atSetpoint() && scoringArm.launchSpeedPIDController.atSetpoint()){
      scoringArm.Launch();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scoringArm.anglePIDController.atSetpoint() && scoringArm.launchSpeedPIDController.atSetpoint();
  }
}
