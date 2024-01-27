// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  
  private final CANSparkFlex m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANcoder m_calibrateCANCoder;

  double drivePIDOutput = 0,turnGoal = 0,driveGoal = 0;

  double setpoint = 0;

  private final SparkPIDController m_drivePIDController;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final SparkPIDController m_turningPIDController;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,int calibrateEncoderChannel) {
    m_driveMotor = new CANSparkFlex(driveMotorChannel,MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel,MotorType.kBrushless);

    m_calibrateCANCoder = new CANcoder(calibrateEncoderChannel);
    
    

    m_driveEncoder = m_driveMotor.getEncoder();
    
    m_turningMotor.setInverted(false);
    m_driveMotor.setInverted(false);
    //m_calibrateCANCoder.configSensorDirection(false);
    m_drivePIDController = m_driveMotor.getPIDController();

    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_drivePIDController.setP(ModuleConstants.kPModuleDriveController);
    m_drivePIDController.setI(ModuleConstants.kIModuleDriveController);
    m_drivePIDController.setD(ModuleConstants.kDModuleDriveController);
    m_drivePIDController.setFF(ModuleConstants.kFFModuleDriveController);
    m_drivePIDController.setIZone(10);
    m_drivePIDController.setIMaxAccum(1,0);
   // m_drivePIDController.setOutputRange(-1, 1);



    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningPIDController = m_turningMotor.getPIDController();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setPositionConversionFactor(1);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveGearRatio*ModuleConstants.kWheelCircumferenceMeters * (1.0/60.0));//ModuleConstants.kDriveConversionFactor * (1/60)    7.8811312001166168504884012775367e-4

    //m_driveEncoder.setVelocityConversionFactor(0.14814814814814814*0.319185813604723 * (1.0/60.0));
    

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setInverted(false);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    m_turningMotor.setInverted(false);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-0.5, 0.5);
    //m_turningPIDController.setConstraints(new Constraints(0.1, 0.1));
    m_turningPIDController.setPositionPIDWrappingMaxInput(Rotation2d.fromDegrees(360).getRotations());
    m_turningPIDController.setPositionPIDWrappingMinInput(Rotation2d.fromDegrees(0).getRotations());
    m_turningPIDController.setPositionPIDWrappingEnabled(true);

    m_turningPIDController.setP(ModuleConstants.kPModuleTurningController);
    m_turningPIDController.setI(ModuleConstants.kIModuleTurningController);
    //m_turningPIDController.setI(0);
    m_turningPIDController.setD(ModuleConstants.kDModuleTurningController);
    m_turningPIDController.setFF(0);
    m_turningPIDController.setOutputRange(-1, 1);

    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Abs Turn Encoder ",  ()->m_calibrateCANCoder.getAbsolutePosition().getValue());
    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Turn Encoder ",  ()->m_turningEncoder.getPosition());
    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Drive Encoder Velocity ",  ()->m_driveEncoder.getVelocity());
    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Drive Encoder Rot",  ()->m_driveEncoder.getPosition()/m_driveEncoder.getPositionConversionFactor());
    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Drive Goal",  ()->driveGoal);
    Shuffleboard.getTab("Debug").addDouble(m_driveMotor.getDeviceId() + " Turn Goal ",  ()->turnGoal);

    
    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), Rotation2d.fromRotations(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        -m_driveEncoder.getPosition(), Rotation2d.fromRotations(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(m_turningEncoder.getPosition()));
        turnGoal = state.angle.getRotations();
        driveGoal = state.speedMetersPerSecond;
        
        //System.out.println(state);

    // Calculate the drive output from the drive PID controller.
    //final double driveOutput =
      //  m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
        //drivePIDOutput = driveOutput;

    // Calculate the turning motor output from the turning PID controller.
    //turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRotations());
    //m_turningMotor.set(turnOutput);
    m_turningPIDController.setReference(state.angle.getRotations(), ControlType.kPosition);
    //turnGoal = state.angle.getRotations();
      //double normalizedDriveOutput = Math.min(Math.max(driveOutput, -0.1), 0.1);
      //drivePIDOutput = normalizedDriveOutput;
    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(-normalizedDriveOutput);
    m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    drivePIDOutput = state.speedMetersPerSecond;
    
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition((m_calibrateCANCoder.getAbsolutePosition().getValue()));
    //m_turningEncoder.setPosition((((m_calibrateCANCoder.getAbsolutePosition().getValue() * 360 + 180) % 360) / 360));
    //setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  public void calibrateTurnEncoders(){
    //m_turningEncoder.setPosition(((m_calibrateCANCoder.getAbsolutePosition()) % 360) / 360);

    m_turningPIDController.setReference(0, ControlType.kPosition);
  }
}
