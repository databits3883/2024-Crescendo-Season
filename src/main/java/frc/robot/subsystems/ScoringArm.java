// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringArmConstants;

public class ScoringArm extends SubsystemBase {

  public CANSparkMax intakeMotor;
  public Servo flapServo = new Servo(ScoringArmConstants.kFlapServoChannel);
  public Servo climbLockServo = new Servo(ScoringArmConstants.kClimbLockServoChannel);

  public CANSparkMax launchMotorLeader;
  public CANSparkMax launchMotorFollower;
  public RelativeEncoder launchSpeedLeaderEncoder;
  public RelativeEncoder launchSpeedFollowerEncoder;
  public SparkPIDController launchSpeedLeaderPIDController;
  public SparkPIDController launchSpeedFollowerPIDController;
  public double launchSpeedSetpoint = 0;

  
  public CANSparkMax armAngleLeaderMotor;
  public CANSparkMax armAngleFollowerMotor1i;
  public CANSparkMax armAngleFollowerMotor2;
  public CANSparkMax armAngleFollowerMotor3i;

  public AbsoluteEncoder absArmAngleEncoder;
  public PIDController anglePIDController = new PIDController(ScoringArmConstants.kAngleP, ScoringArmConstants.kAngleI, ScoringArmConstants.kAngleD);

  enum ArmStates {
    PICKUP,
    AMP,
    STATICSPEAKER,
    DYNAMICSPEAKER,
    PREPCLIMB,
    DONECLIMB
  }

  /** Creates a new ScoringArm. */
  public ScoringArm() {

    armAngleLeaderMotor = new CANSparkMax(ScoringArmConstants.kArmAngleMotor1ID, MotorType.kBrushless);
    //armAngleFollowerMotor1 = new CANSparkMax(ScoringArmConstants.kArmAngleMotor2ID, MotorType.kBrushless);
    //armAngleFollowerMotor2i = new CANSparkMax(ScoringArmConstants.kArmAngleMotor3iID, MotorType.kBrushless);
    //armAngleFollowerMotor3i = new CANSparkMax(ScoringArmConstants.kArmAngleMotor4iID, MotorType.kBrushless);

    armAngleLeaderMotor.setInverted(true);
    //armAngleFollowerMotor1.follow(armAngleLeaderMotor, true);
    //armAngleFollowerMotor2i.follow(armAngleLeaderMotor, false);
    //armAngleFollowerMotor3i.follow(armAngleLeaderMotor, false);

    launchMotorLeader = new CANSparkMax(ScoringArmConstants.kLaunchMotorLeaderID, MotorType.kBrushless);
    launchMotorFollower = new CANSparkMax(ScoringArmConstants.kLaunchMotorFollowerID, MotorType.kBrushless);
    launchSpeedLeaderEncoder = launchMotorLeader.getEncoder();
    launchSpeedFollowerEncoder = launchMotorFollower.getEncoder();
    
    intakeMotor = new CANSparkMax(ScoringArmConstants.kIntakeMotorID, MotorType.kBrushless);

    absArmAngleEncoder = armAngleLeaderMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    absArmAngleEncoder.setPositionConversionFactor(360);
    absArmAngleEncoder.setVelocityConversionFactor(360);//degrees per second


    anglePIDController.setP(ScoringArmConstants.kAngleP);
    anglePIDController.setI(ScoringArmConstants.kAngleI);
    anglePIDController.setD(ScoringArmConstants.kAngleD);
    anglePIDController.setIZone(ScoringArmConstants.kAngleIZone);
    anglePIDController.setTolerance(ScoringArmConstants.kAnglePosTolerance,ScoringArmConstants.kAngleVelTolerance);
    anglePIDController.setSetpoint(absArmAngleEncoder.getPosition());
    anglePIDController.enableContinuousInput(0, 360);
    

    launchSpeedLeaderPIDController = launchMotorLeader.getPIDController();
    launchSpeedFollowerPIDController = launchMotorFollower.getPIDController();

    launchSpeedLeaderEncoder.setPositionConversionFactor(4*Math.PI);//diameter of wheel times pi
    launchSpeedLeaderEncoder.setVelocityConversionFactor(4*0.254*Math.PI/60);

    launchSpeedFollowerEncoder.setPositionConversionFactor(4*Math.PI);//diameter of wheel times pi
    launchSpeedFollowerEncoder.setVelocityConversionFactor(4*0.254*Math.PI/60);
    
    launchSpeedLeaderPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedLeaderPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedLeaderPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedLeaderPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedLeaderPIDController.setFF(ScoringArmConstants.kLaunchSpeedFF);

    launchSpeedFollowerPIDController.setP(ScoringArmConstants.kLaunchSpeedP);
    launchSpeedFollowerPIDController.setI(ScoringArmConstants.kLaunchSpeedI);
    launchSpeedFollowerPIDController.setD(ScoringArmConstants.kLaunchSpeedD);
    launchSpeedFollowerPIDController.setIZone(ScoringArmConstants.kLaunchSpeedIZone);
    launchSpeedFollowerPIDController.setFF(ScoringArmConstants.kLaunchSpeedFF);
    
    
    //Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Error", launchSpeedPIDController::getPositionError);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel SP", ()-> launchSpeedSetpoint);
    Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Encoder", launchSpeedLeaderEncoder::getVelocity);
    //Shuffleboard.getTab("Arm Debug").addDouble("Launch Vel Output", () -> launchSpeedPIDController.calculate(10));

    Shuffleboard.getTab("Arm Debug").addDouble("Arm SP", anglePIDController::getSetpoint);

    SetLaunchSpeed(0);//theoretical max of 622.9 meters per second
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    RunAnglePIDControl();

    
    RunLaunchSpeedPIDControl();
    //launchMotorLeader.set(0.1);

  }

  public void RunAnglePIDControl(){
    SetArmAngleMotors(anglePIDController.calculate(absArmAngleEncoder.getPosition()));
  }

  public void SetArmAngleMotors(double fraction){
    armAngleLeaderMotor.set(fraction);
  }

  public void SetArmAngle(double armDeg){
    anglePIDController.setSetpoint(armDeg%360);
  }

  public void ChangeArmAngle(double deg){
    SetArmAngle(anglePIDController.getSetpoint() + deg);
  }

  public double GetArmAngle(){
    return absArmAngleEncoder.getPosition();
  }

  public void RunLaunchSpeedPIDControl(){
    //launchMotorLeader.set(launchSpeedPIDController.calculate(launchSpeedEncoder.getVelocity()));
    launchSpeedLeaderPIDController.setReference(launchSpeedSetpoint, ControlType.kVelocity);
    launchSpeedFollowerPIDController.setReference(-1 * launchSpeedSetpoint, ControlType.kVelocity);
  }

  public void SetLaunchSpeed(double launchRPM){
    launchSpeedSetpoint = launchRPM;
  }

  public void ChangeLaunchSpeed(double deltaRPM){
    SetLaunchSpeed(launchSpeedSetpoint+deltaRPM);
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
    intakeMotor.set(1);
  }

  public void SetFlap(double pos){
    flapServo.set(pos);
  }

public void StopIntake() {
    intakeMotor.set(0);
}

  public boolean atLaunchSetpoint() {
    return (Math.abs(launchSpeedSetpoint-launchSpeedLeaderEncoder.getVelocity()) < 5);
  }

  public void UnlatchClimb(){
    climbLockServo.set(0);
  }

  public void LatchClimb(){
    climbLockServo.set(1);
  }

  public void PrepareClimb(){
    UnlatchClimb();
    SetArmAngle(ScoringArmConstants.kArmPosClimbPrep);
  }

  public void Climb(){
    SetArmAngle(ScoringArmConstants.kArmPosClimbFinish);
  }

  public void GoToPickupPos(){
    SetArmAngle(ScoringArmConstants.kArmPosPickup);
  }

}
