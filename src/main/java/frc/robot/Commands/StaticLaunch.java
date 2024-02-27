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
  double launchAngle;
  Timer timer;

  /** Creates a new StaticLaunch. */
  public StaticLaunch(ScoringArm arm,double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ScoringArm = arm;
    launchAngle = angle;
    launchSpeed = 250;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ScoringArm.SetArmAngle(launchAngle);
    m_ScoringArm.OutakeToSensorFast();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ScoringArm.CoastLaunchMotors();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSPs = (m_ScoringArm.anglePIDController.getPositionError() < 5);
    if(atSPs){
      timer.start();
    }

    if(timer.hasElapsed(1.0) && m_ScoringArm.atLaunchSetpoint()){
      m_ScoringArm.Launch();
    }
    else if(timer.hasElapsed(0.5)){
      m_ScoringArm.SetLaunchSpeed(launchSpeed);
    }

    return timer.hasElapsed(1.5);
  }
}
