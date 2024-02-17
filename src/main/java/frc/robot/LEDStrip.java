// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class LEDStrip {
    AddressableLED addressableLED = new AddressableLED(0); //should be PWM location

    
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

    public LEDStrip(){

    addressableLED.setLength(ledBuffer.getLength());

    // Set the data
    addressableLED.setData(ledBuffer);
    addressableLED.start();
    }



}
