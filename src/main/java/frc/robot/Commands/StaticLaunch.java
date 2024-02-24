// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.ScoringArm;

public class StaticLaunch extends Command {
  ScoringArm m_ScoringArm;
  double launchSpeed;
  Timer timer;

  /** Creates a new StaticLaunch. */
  public StaticLaunch(ScoringArm arm,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ScoringArm = arm;
    launchSpeed = speed;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosNearStaticLaunch);
    m_ScoringArm.SetLaunchSpeed(launchSpeed);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ScoringArm.Launch();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSPs = m_ScoringArm.atLaunchSetpoint() && (m_ScoringArm.anglePIDController.getPositionError() < 5);
    if(atSPs){
      timer.start();
    }
    return timer.hasElapsed(0.25);
  }
}
