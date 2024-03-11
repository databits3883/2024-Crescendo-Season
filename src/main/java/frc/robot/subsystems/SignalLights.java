// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class SignalLights extends SubsystemBase {
  public AddressableLED armLEDs;
  public AddressableLEDBuffer armLEDBuffer = new AddressableLEDBuffer(LEDConstants.kArmLEDCount);
  /** Creates a new SignalLights. */
  public SignalLights() {
    armLEDs = new AddressableLED(LEDConstants.kArmLEDPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetArmLEDBufferToSolidColor(int h, int s, int v){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setHSV(i, h, s, v);
   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToCoolAnimation(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      
      armLEDBuffer.setHSV(i, 120, 61, (int)(55 * Math.sin(i + ((double)(System.currentTimeMillis()*100)))) );
   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToAllianceColor(BooleanSupplier isBlueSupplier){

    boolean isBlue = isBlueSupplier.getAsBoolean();
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {

      if(isBlue){
        armLEDBuffer.setRGB(i, 0, 0,255);
      }
      else{
        armLEDBuffer.setRGB(i, 255, 0,0);
      }

   }
   ApplyLEDBuffer();
  }

  public void SetArmLEDBufferToDatabitsColors(){
    for (var i = 0; i < armLEDBuffer.getLength(); i++) {
      armLEDBuffer.setHSV(i, 120, 61, 55);
    }
    ApplyLEDBuffer();
  }

  public void ApplyLEDBuffer(){
    armLEDs.setData(armLEDBuffer);
  }
}
