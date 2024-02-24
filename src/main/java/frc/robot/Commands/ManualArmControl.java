// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringArm;

public class ManualArmControl extends Command {

    public ScoringArm scoringArm;
    public DoubleSupplier angleSupplier;

  /** Creates a new ManualArmControl. */
  public ManualArmControl(ScoringArm arm, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    angleSupplier=angle;
    scoringArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scoringArm.SetArmAngle(angleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
