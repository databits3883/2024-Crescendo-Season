// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class SignalLights extends SubsystemBase {
  
  public AddressableLED armLEDs;
  public AddressableLEDBuffer armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
  public boolean sensorWasBlocked = false;
  public boolean visualizingIntakeSensor = false;
  public boolean idleAnimation = true;
  public Timer animationTimer = new Timer();

  public LightSignal currentSignal = LightSignal.databits;

  public enum LightSignal {
    noNote,
    hasNote,
    intaking,
    launchPrep,
    launchReady,
    climbPrep,
    climbFinish,
    databits
  }

  /** Creates a new SignalLights. */
  public SignalLights() {
    armLEDs = new AddressableLED(LEDConstants.kArmLEDPort);
    animationTimer.start();

    armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
    SetArmLEDBufferToSolidColor(LEDConstants.kDatabitsColor);
    armLEDs.setLength(LEDConstants.kArmLEDCount);
    armLEDs.setData(armLEDBuffer);
    armLEDs.start();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentSignal) {
      case noNote:
        SetArmLEDBufferToSolidColor(LEDConstants.kNoNoteColor);
        break;
      case hasNote:
        SetArmLEDBufferToSolidColor(LEDConstants.kHasNoteColor);
        break;
      case intaking:
        BlinkColorWithTime(LEDConstants.kIntakeColor, LEDConstants.kOffColor, animationTimer.get());
        break;
      case launchPrep:
        BlinkColorWithTime(LEDConstants.kLaunchPrepColor, LEDConstants.kOffColor, animationTimer.get());
        break;
      case launchReady:
        SetArmLEDBufferToSolidColor(LEDConstants.kLaunchReadyColor);
        break;
      case climbPrep:
        BlinkColorWithTime(LEDConstants.kClimbReadyColor, LEDConstants.kOffColor, animationTimer.get());
        break;
      case climbFinish:
        SetArmLEDBufferToSolidColor(LEDConstants.kClimbColor);
        break;
      case databits:
        SetArmLEDBufferToSolidColor(LEDConstants.kDatabitsColor);
        break;
    
      default:
        SetArmLEDBufferToSolidColor(LEDConstants.kErrorColor);
        break;
    }

    armLEDs.setData(armLEDBuffer);
    armLEDs.start();
  }

  public void SetArmLEDBufferToSolidColor(Color8Bit color){

    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setLED(i, color);
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

  public void BlinkColorWithTime(Color8Bit activeColor , Color8Bit inactiveColor, double timer){
    if(timer % 0.5 > 0.25){
      SetArmLEDBufferToSolidColor(inactiveColor);
    }
    else{
      SetArmLEDBufferToSolidColor(activeColor);
    }
  }


  
}
