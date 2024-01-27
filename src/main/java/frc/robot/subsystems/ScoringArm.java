// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringArmConstants;

public class ScoringArm extends SubsystemBase {

  public CANSparkMax intakeMotor = new CANSparkMax(ScoringArmConstants.kIntakeMotorID, MotorType.kBrushless);

  public CANSparkMax launchMotorLeader = new CANSparkMax(ScoringArmConstants.kLaunchMotorLeaderID, MotorType.kBrushless);
  public CANSparkMax launchMotorFollower = new CANSparkMax(ScoringArmConstants.kLaunchMotorFollowerID, MotorType.kBrushless);
  public PIDController launchSpeedPIDController = new PIDController(ScoringArmConstants.kLaunchSpeedP,ScoringArmConstants.kLaunchSpeedI,ScoringArmConstants.kLaunchSpeedD);


  public AbsoluteEncoder absArmAngleEncoder;
  public CANSparkMax armAngleMotorLeader = new CANSparkMax(ScoringArmConstants.kArmAngleMotorLeader, MotorType.kBrushless);
  public CANSparkMax armAngleMotorFollower = new CANSparkMax(ScoringArmConstants.kArmAngleMotorFollower, MotorType.kBrushless);
  public PIDController anglePidController = new PIDController(ScoringArmConstants.kAngleP, ScoringArmConstants.kAngleI, ScoringArmConstants.kAngleD);

  /** Creates a new ScoringArm. */
  public ScoringArm() {

    armAngleMotorFollower.follow(armAngleMotorLeader);
    absArmAngleEncoder = armAngleMotorLeader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absArmAngleEncoder.setPositionConversionFactor(360);
    absArmAngleEncoder.setVelocityConversionFactor(1);


    anglePidController.setP(ScoringArmConstants.kAngleP);
    anglePidController.setI(ScoringArmConstants.kAngleI);
    anglePidController.setD(ScoringArmConstants.kAngleD);
    anglePidController.setIZone(ScoringArmConstants.kAngleIZone);
    anglePidController.setTolerance(ScoringArmConstants.kAnglePosTolerance,ScoringArmConstants.kAngleVelTolerance);
    anglePidController.setSetpoint(absArmAngleEncoder.getPosition());

    anglePidController.setP(ScoringArmConstants.kLaunchSpeedP);
    anglePidController.setI(ScoringArmConstants.kLaunchSpeedI);
    anglePidController.setD(ScoringArmConstants.kLaunchSpeedD);
    anglePidController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    anglePidController.setTolerance(ScoringArmConstants.kLaunchSpeedPosTolerance,ScoringArmConstants.kLaunchSpeedVelTolerance);
    anglePidController.setSetpoint(0);//tell the launch motors to start at 0 speed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!anglePidController.atSetpoint()) {
      RunAnglePIDControl();
    }
  }

  public void RunAnglePIDControl(){
    armAngleMotorFollower.set(anglePidController.calculate(absArmAngleEncoder.getPosition()));
  }

  public void SetArmAngle(double armDeg){
    anglePidController.setSetpoint(armDeg);
  }

  public void ChangeArmAngle(double deg){
    SetArmAngle(absArmAngleEncoder.getPosition() + deg);
  }

  public void RunLaunchSpeedPIDControl(){
    launchMotor1.set(launchSpeedPIDController.calculate());
  }

  public void SetLaunchSpeed(double armDeg){
 
  }

  public void ChangeLaunchSpeed(double deg){

  }
}
