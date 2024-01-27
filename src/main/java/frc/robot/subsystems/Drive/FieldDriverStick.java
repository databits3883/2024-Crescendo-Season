// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

/** Add your docs here. */
public class FieldDriverStick {

    Joystick m_Joystick;
    public double oldXInput = 0, oldYInput = 0;

    public FieldDriverStick(Joystick joystick){
        m_Joystick = joystick;
    }

    public double getX(){
        double output = m_Joystick.getY();
        //output *= -1;
        //output = ReduceDecceleration(output, oldXInput);
        //output = Math.pow(output, OIConstants.kDriveStickPower) * Math.signum(output);
        if(Math.abs(output) < 0.05){
            output = 0;
        }

        //output = ReduceDecceleration(output, oldXInput);
        oldXInput = output;
        // return output * DriveConstants.kMaxSpeedMetersPerSecond;
        return output;
    }

    public double getY(){
        double output = m_Joystick.getX();
        //output *= -1;
        
        //output = Math.pow(output, OIConstants.kDriveStickPower) * Math.signum(output);
        if(Math.abs(output) < 0.05){
            output = 0;
        }

        //output = ReduceDecceleration(output, oldYInput);
        oldYInput = output;
        // return output * DriveConstants.kMaxSpeedMetersPerSecond;
        return output;
    }

    public double getZ(){
        double output = m_Joystick.getTwist();
        //output = Math.pow(output, OIConstants.kDriveStickPower) * Math.signum(output);
        output *= -1;
        if(Math.abs(output) < 0.05){
            output = 0;
        }
        
        // return output * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
        return output;
    }

    public double ReduceDecceleration(double newInput, double oldInput){
        double output = 0;
        if (Math.abs(newInput) < Math.abs(oldInput)){
            output = oldInput + 0.1 * (newInput - oldInput);;
        }
        else{
            output = newInput;
        }
        return output;
    }
}

