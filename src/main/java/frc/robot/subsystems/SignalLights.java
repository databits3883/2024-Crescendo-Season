// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class SignalLights extends SubsystemBase {
  public AddressableLED armLEDs;
  public ScoringArm scoringArm;
  public AddressableLEDBuffer armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
  public boolean sensorWasBlocked = false;
  public boolean visualizingIntakeSensor = false;
  public boolean idleAnimation = true;
  public Timer animationTimer = new Timer();
  /** Creates a new SignalLights. */
  public SignalLights(ScoringArm arm) {
    armLEDs = new AddressableLED(LEDConstants.kArmLEDPort);
    scoringArm = arm;
    animationTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean sensorBlocked = scoringArm.IntakeSensorBlocked();
    if (visualizingIntakeSensor && sensorWasBlocked != sensorBlocked) {
      if(sensorBlocked){
        SetArmLEDBufferToSolidColorHSV(68, 204, 53);
      }
      else{
        SetArmLEDBufferToSolidColorHSV(189, 53, 204);
      }
      
    }
  }

  public void SetArmLEDBufferToSolidColorHSV(int h, int s, int v){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setHSV(i, h%180, s%255, v%255);
   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToCoolAnimation(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setRGB(i, 68, 53, (int)Math.round(204 * Math.sin(i + (animationTimer.get()))) );
   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToAllianceColor(BooleanSupplier isBlueSupplier){

    boolean isBlue = isBlueSupplier.getAsBoolean();
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {

      if(isBlue){
        armLEDBuffer.setRGB(i, 0, 0,128);
      }
      else{
        armLEDBuffer.setRGB(i, 128, 0,0);
      }

   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToDatabitsColors(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      armLEDBuffer.setHSV(i, 68, 204, 53);
    }
    ApplyLEDBuffer();
  }

  public void ApplyLEDBuffer(){
    armLEDs.setData(armLEDBuffer);
  }
}
