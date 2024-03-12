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

  public LightSignal currentSignal = LightSignal.noNote;

  enum LightSignal {
    noNote,
    hasNote,
    intaking,
    launching,
    climbPrep,
    climbFinish
  }

  /** Creates a new SignalLights. */
  public SignalLights(ScoringArm arm) {
    armLEDs = new AddressableLED(LEDConstants.kArmLEDPort);
    scoringArm = arm;
    animationTimer.start();

    armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
    armLEDs.setLength(LEDConstants.kArmLEDCount);
    armLEDs.setData(armLEDBuffer);
    armLEDs.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void SetArmLEDBufferToSolidColorHSV(int h, int s, int v){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setHSV(i, h%180, s%255, v%255);
   }
    
  }

  public void SetArmLEDBufferToCoolAnimation(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setRGB(i, 68, 53, (int)Math.round(204 * Math.sin(i + (animationTimer.get()))) );
   }
    
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
  }

  public void SetArmLEDBufferToDatabitsColors(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      armLEDBuffer.setHSV(i, 68, 204, 53);
    }
  }

  public void Signal(LightSignal newSignal){
    currentSignal = newSignal;
  }


  
}
