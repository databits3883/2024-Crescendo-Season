// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSystem extends SubsystemBase {

  public double armBalance = 0.5; //determines which arm should pull harder in order to keep the robot balanced while climbing
  public PIDController balancePidController = new PIDController(0, 0, 0);
  
  public CANSparkMax leftArmMotor;
  public CANSparkMax rightArmMotor;

  /** Creates a new ClimbSystem. */
  public ClimbSystem() {
    //leftArmMotor = new CANSparkMax(Constants.ClimbConstants.kLeftArmMotorChannel, MotorType.kBrushed);
    //rightArmMotor = new CANSparkMax(Constants.ClimbConstants.kRightArmMotorChannel, MotorType.kBrushed);

    balancePidController.setSetpoint(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void BalanceRobotWithClimb(double rollAngleDeg){
    armBalance = (balancePidController.calculate(rollAngleDeg) + 1) / 2;

    leftArmMotor.set((1-armBalance));
    rightArmMotor.set(armBalance);
  }

}
