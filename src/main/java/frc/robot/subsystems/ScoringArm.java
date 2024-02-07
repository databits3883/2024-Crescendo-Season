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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringArmConstants;

public class ScoringArm extends SubsystemBase {

  public CANSparkMax intakeMotor;
  public Servo flapServo = new Servo(ScoringArmConstants.kFlapServo1Channel);

  public CANSparkMax launchMotorLeader;
  public RelativeEncoder launchSpeedEncoder;
  public PIDController launchSpeedPIDController = new PIDController(ScoringArmConstants.kLaunchSpeedP,ScoringArmConstants.kLaunchSpeedI,ScoringArmConstants.kLaunchSpeedD);
  public double launchSpeedSetpoint = 0;

  public AbsoluteEncoder absArmAngleEncoder;
  public CANSparkMax armAngleMotorLeader;
  public PIDController anglePIDController = new PIDController(ScoringArmConstants.kAngleP, ScoringArmConstants.kAngleI, ScoringArmConstants.kAngleD);

  /** Creates a new ScoringArm. */
  public ScoringArm() {

    armAngleMotorLeader = new CANSparkMax(ScoringArmConstants.kArmAngleMotorLeader, MotorType.kBrushless);

    launchMotorLeader = new CANSparkMax(ScoringArmConstants.kLaunchMotorLeaderID, MotorType.kBrushless);
    launchSpeedEncoder = launchMotorLeader.getEncoder();
    
    intakeMotor = new CANSparkMax(ScoringArmConstants.kIntakeMotorID, MotorType.kBrushless);

    absArmAngleEncoder = armAngleMotorLeader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absArmAngleEncoder.setPositionConversionFactor(1);
    absArmAngleEncoder.setVelocityConversionFactor(1);//degrees per second


    anglePIDController.setP(ScoringArmConstants.kAngleP);
    anglePIDController.setI(ScoringArmConstants.kAngleI);
    anglePIDController.setD(ScoringArmConstants.kAngleD);
    anglePIDController.setIZone(ScoringArmConstants.kAngleIZone);
    anglePIDController.setTolerance(ScoringArmConstants.kAnglePosTolerance,ScoringArmConstants.kAngleVelTolerance);
    anglePIDController.setSetpoint(absArmAngleEncoder.getPosition());

    launchSpeedEncoder.setPositionConversionFactor(4*Math.PI);//diameter of wheel times pi
    launchSpeedEncoder.setVelocityConversionFactor(4*Math.PI/60);
    

    launchSpeedPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedPIDController.setTolerance(ScoringArmConstants.kLaunchSpeedPosTolerance,ScoringArmConstants.kLaunchSpeedVelTolerance);
    launchSpeedPIDController.disableContinuousInput();
    launchSpeedPIDController.setSetpoint(80);//tell the launch motors to start at 0 speed
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Error", launchSpeedPIDController::getPositionError);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel SP", launchSpeedPIDController::getSetpoint);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Encoder", launchSpeedEncoder::getVelocity);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Output", () -> launchSpeedPIDController.calculate(10));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!anglePIDController.atSetpoint()) {
      RunAnglePIDControl();//runs PID control if the motor is not at the setpoint
    }
    else{
      armAngleMotorLeader.set(0);//stops the motor if you are at the setpoint
    }

    
    RunLaunchSpeedPIDControl();
    

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
    flapServo.set(pos);
  }

public void StopIntake() {
    intakeMotor.set(0);
}
}
