// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
@Logged
public class LED extends SubsystemBase{

    public final AddressableLED led = new AddressableLED(0);
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(26);

    
    public final LEDPattern purplePattern = LEDPattern.solid(Color.kPurple);
    public final LEDPattern dimPurplePattern = purplePattern.atBrightness(Percent.of(10));
    public final LEDPattern greenPattern = LEDPattern.solid(Color.kGreen);
    public final LEDPattern dimgreenPattern = greenPattern.atBrightness(Percent.of(10));
    public final LEDPattern goldPattern = LEDPattern.solid(new Color(135,110,0));
    public final LEDPattern dimGoldPattern = goldPattern.atBrightness(Percent.of(50));
    public final LEDPattern rainbowPattern = LEDPattern.rainbow(255,255);
    public final LEDPattern rainbowScroll = rainbowPattern.scrollAtAbsoluteSpeed(FeetPerSecond.of(0.5), Feet.of(1.0/12.0));
    public final LEDPattern dimRainbowPattern = rainbowScroll.atBrightness(Percent.of(10));
    public  LEDPattern patternToApply = dimRainbowPattern;

    public LED () {

        led.setLength(ledBuffer.getLength());
        patternToApply.applyTo(ledBuffer);
        led.setData(ledBuffer);
        led.start();





    }
    public void periodic (){
        
        patternToApply.applyTo(ledBuffer);
        led.setData(ledBuffer);

    }

    public void greenlight () {
     patternToApply = dimgreenPattern;
    }

    public void purplelight () {
        patternToApply = dimPurplePattern;
       }

    public void goldlight () {
        patternToApply = dimGoldPattern;
       }

    public void rainbowlight () {
      patternToApply = dimRainbowPattern;
    }










}
