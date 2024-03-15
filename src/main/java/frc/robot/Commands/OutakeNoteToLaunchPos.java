// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.ScoringArm;

public class OutakeNoteToLaunchPos extends Command {

  ScoringArm scoringArm;
  
  /** Creates a new OutakeNoteToLaunchPos. */
  public OutakeNoteToLaunchPos(ScoringArm arm) {
    scoringArm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoringArm.SetArmAngle(ScoringArmConstants.kArmPosNearStaticLaunch);
    scoringArm.CoastLaunchMotors();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scoringArm.SetIntakeMotors(-0.3,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scoringArm.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !scoringArm.IntakeSensorBlocked();
  }
}
