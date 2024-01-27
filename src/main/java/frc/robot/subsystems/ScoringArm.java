// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringArmConstants;

public class ScoringArm extends SubsystemBase {

  public CANSparkMax intakeMotor;
  public Servo flapServo1 = new Servo(ScoringArmConstants.kFlapServo1Channel);
  public Servo flapServo2 = new Servo(ScoringArmConstants.kFlapServo2Channel);

  public CANSparkMax launchMotorLeader;
  public CANSparkMax launchMotorFollower;
  public RelativeEncoder launchSpeedEncoder;
  public PIDController launchSpeedPIDController = new PIDController(ScoringArmConstants.kLaunchSpeedP,ScoringArmConstants.kLaunchSpeedI,ScoringArmConstants.kLaunchSpeedD);


  public AbsoluteEncoder absArmAngleEncoder;
  public CANSparkMax armAngleMotorLeader;
  public CANSparkMax armAngleMotorFollower;
  public PIDController anglePIDController = new PIDController(ScoringArmConstants.kAngleP, ScoringArmConstants.kAngleI, ScoringArmConstants.kAngleD);

  /** Creates a new ScoringArm. */
  public ScoringArm() {

    //armAngleMotorLeader = new CANSparkMax(ScoringArmConstants.kArmAngleMotorLeader, MotorType.kBrushless);
    //armAngleMotorFollower = new CANSparkMax(ScoringArmConstants.kArmAngleMotorFollower, MotorType.kBrushless);
    //launchMotorLeader = new CANSparkMax(ScoringArmConstants.kLaunchMotorLeaderID, MotorType.kBrushless);
    //launchMotorFollower = new CANSparkMax(ScoringArmConstants.kLaunchMotorFollowerID, MotorType.kBrushless);
    //intakeMotor = new CANSparkMax(ScoringArmConstants.kIntakeMotorID, MotorType.kBrushless);

    //armAngleMotorFollower.follow(armAngleMotorLeader);
    //absArmAngleEncoder = armAngleMotorLeader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    //absArmAngleEncoder.setPositionConversionFactor(360);
    //absArmAngleEncoder.setVelocityConversionFactor(1);


    anglePIDController.setP(ScoringArmConstants.kAngleP);
    anglePIDController.setI(ScoringArmConstants.kAngleI);
    anglePIDController.setD(ScoringArmConstants.kAngleD);
    anglePIDController.setIZone(ScoringArmConstants.kAngleIZone);
    anglePIDController.setTolerance(ScoringArmConstants.kAnglePosTolerance,ScoringArmConstants.kAngleVelTolerance);
    //anglePIDController.setSetpoint(absArmAngleEncoder.getPosition());

    launchSpeedPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedPIDController.setTolerance(ScoringArmConstants.kLaunchSpeedPosTolerance,ScoringArmConstants.kLaunchSpeedVelTolerance);
    launchSpeedPIDController.setSetpoint(0);//tell the launch motors to start at 0 speed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //if (!anglePIDController.atSetpoint()) {
    //  RunAnglePIDControl();//runs PID control if the motor is not at the setpoint
    //}
    //else{
    //  armAngleMotorLeader.set(0);//stops the motor if you are at the setpoint
    //}

    
    //RunLaunchSpeedPIDControl();
    

  }

  public void RunAnglePIDControl(){
    armAngleMotorLeader.set(anglePIDController.calculate(absArmAngleEncoder.getPosition()));
  }

  public void SetArmAngle(double armDeg){
    anglePIDController.setSetpoint(armDeg);
  }

  public void ChangeArmAngle(double deg){
    SetArmAngle(absArmAngleEncoder.getPosition() + deg);
  }

  public void RunLaunchSpeedPIDControl(){
    launchMotorLeader.set(launchSpeedPIDController.calculate(launchSpeedEncoder.getVelocity()));
  }

  public void SetLaunchSpeed(double launchRPM){
    launchSpeedPIDController.setSetpoint(launchRPM);
  }

  public void ChangeLaunchSpeed(double deltaRPM){
    SetLaunchSpeed(launchSpeedPIDController.getSetpoint()+deltaRPM);
  }

  public void Intake(){
    SetFlap(0);
    intakeMotor.set(0.1);
  }

  public void Outtake(){
    intakeMotor.set(-0.1);
  }

  public void Launch(){
    SetFlap(0.1);
    intakeMotor.set(0.1);
  }

  public void SetFlap(double pos){
    flapServo1.set(pos);
    flapServo2.set(pos);
  }
}
