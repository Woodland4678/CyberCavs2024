// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
/** 
 * A singleton class to control LED strips. Singleton means we allow only one instance of the class.
 */
public class LEDStrip {
    
    final int NUM_LEDS = 60;    
    private int rainbowFirstHue = 0;
    public static enum LEDModes {
        OFF,
        SOLIDGREEN,
        BLINKGREEN,
        SOLIDRED,
        SOLIDBLUE,
        RAINBOW,
        ROBOTDISABLEDPATTERN
      }
    
    LEDModes LEDMode;

    private static final LEDStrip instance = new LEDStrip();
    AddressableLED addressableLED = new AddressableLED(0); //should be PWM location
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
    private double intervalSeconds= 0.5; 
	private boolean blinkLEDon = true;
	private double lastChange = 0;
    private int diagnosticPattern;
    
    // diagnostic segments 
    /** Left segment 1 (starts at the top)*/  
    public static final int elbowDiag = 0X01;
    /** Left segment 2 */
    public static final int shoulderDiag = 0X02;
    /** Left segment 3*/
    public static final int gyroDiag = 0X04;
    /** Left segment 4 */
    public static final int limelightDiag = 0X08;
    /** Left segment 5 (bottom) */
    public static final int apriltagDiag = 0X10;
    /** Right segment 1 (starts at bottom) */
    public static final int swerve1Diag = 0X20; // needs position on robot?
    /** Right segment 2 */
    public static final int swerve2Diag = 0X40; // ^
    /** Right segment 3*/
    public static final int swerve3Diag = 0X80; // ^
    /** Right segment 4 */
    public static final int swerve4Diag = 0X100; // ^
    /** All segments blue */
    public static final int allClear = 0X1FF;

    // private constructor so clients can't use it
    private LEDStrip(){
        addressableLED.setLength(ledBuffer.getLength());

        // Set the data
        addressableLED.setData(ledBuffer);
        addressableLED.start();
        LEDMode = LEDModes.OFF;
    }

    
    public static LEDStrip getInstance(){
        return instance;
    }
    
    private void setColour(int r,int g, int b){

        for (int index = 0; index < ledBuffer.getLength(); index++){
			ledBuffer.setRGB(index, r, g, b);
		}
		
    }

    private void blinkLEDs(int r, int g,int b){
        double timestamp = Timer.getFPGATimestamp();
		if (timestamp- lastChange > intervalSeconds){
		    blinkLEDon = !blinkLEDon;
			lastChange = timestamp;
		}
		if (blinkLEDon){
			setColour(r, g, b);
		} else {
			setColour(0, 0, 0);
		}
    }

    private void rainbow(){
        int currentHue;
		for (int index = 0; index < ledBuffer.getLength(); index++){
			currentHue = (rainbowFirstHue + (index * 180 / ledBuffer.getLength())) % 180;
			ledBuffer.setHSV(index, currentHue, 255, 128);
		}

		rainbowFirstHue = (rainbowFirstHue + 3) % 180;
    }

    /*
     *  For the diagnostic checks, we split diagnostics across 2 strips of 30 leds.
     *  Strips were divided into segments of LEDs, 1 segment per diagnostic, none spanning over multiple strips.
     *  
     */
    public void diagnosticLEDmode(){
        if (diagnosticPattern == allClear) { // if all is good, go entirely blue
	        setLEDMode(LEDModes.SOLIDBLUE);
        }
	    else { // Something other than "all is well".  Light up the required segments that have a 0 bit in bval
            setLEDMode(LEDModes.ROBOTDISABLEDPATTERN);
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                // Set the value
                ledBuffer.setRGB(i, 255, 0, 0); // Everything is red to start with.
            }

            // First strip of LEDS, 6 leds per diagnostic state, 
            // represents shoulder/elbow/gryo/limelight/april tag camera (not necessarily in that order) 
            if((diagnosticPattern & elbowDiag) != 0) {
                for(int i = 0;i<=5;i++)
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & shoulderDiag) != 0) {
                for(int i = 6;i<=11;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & gyroDiag) != 0) {
                for(int i = 12;i<=17;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & limelightDiag) != 0) {
                for(int i = 18;i<=23;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & apriltagDiag) != 0) {
                for(int i = 24;i<=29;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            // Second LED strip, 7 or 8 leds per diagnostic check
            // represents each each swerve module
            if((diagnosticPattern & swerve1Diag) != 0) {
                for(int i = 30;i<=36;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve2Diag) != 0) {
                for(int i = 37;i<=44;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve3Diag) != 0) {
                for(int i = 45;i<=51;i++) 
                    ledBuffer.setRGB(i, 0, 255, 0); 
            }
            if((diagnosticPattern & swerve4Diag) != 0){
                for(int i = 52; i<=59;i++)
                    ledBuffer.setRGB(i, 0, 255, 0);
            }
	    }
    }

    public void setDiagnosticPattern(int binaryVal){
        diagnosticPattern = binaryVal;
    }

    public void setLEDMode(LEDModes inputLEDMode){
        LEDMode = inputLEDMode;

            switch(LEDMode){
            case OFF:
                setColour(0,0,0);
                break;
            case SOLIDGREEN:
                setColour(0, 255, 0);
                break;
            case SOLIDRED:
                setColour(255, 0, 0);
                break;
            case SOLIDBLUE:
                setColour(0, 0, 255);
                break;
            case BLINKGREEN:
                blinkLEDs(0, 255, 0);
                break;
            case RAINBOW:
                rainbow();
                break;
            case ROBOTDISABLEDPATTERN:
                diagnosticLEDmode();
                break;
        }
        addressableLED.setData(ledBuffer);
    }
/*
    public void periodic(){
        switch(LEDMode){
            case OFF:
                setColour(0,0,0);
                break;
            case SOLIDGREEN:
                setColour(0, 255, 0);
                break;
            case SOLIDRED:
                setColour(255, 0, 0);
                break;
            case SOLIDBLUE:
                setColour(0, 0, 255);
                break;
            case BLINKGREEN:
                blinkLEDs(0, 255, 0);
                break;
            case RAINBOW:
                rainbow();
                break;
            case ROBOTDISABLEDPATTERN:
                diagnosticLEDmode();
                break;
        }
        addressableLED.setData(ledBuffer);
    }
    */
}
