// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Constructor;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
@Logged
public class LED extends SubsystemBase{

    public final AddressableLED led = new AddressableLED(0);
    public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(26);

    
    public final LEDPattern purplePattern = LEDPattern.solid(Color.kPurple);
    public final LEDPattern greenPattern = LEDPattern.solid(Color.kGreen);
    public final LEDPattern goldPattern = LEDPattern.solid(new Color(135,110,0));
    public final LEDPattern dimGoldPattern = goldPattern.atBrightness(Percent.of(50));
    public final LEDPattern rainbowPattern = LEDPattern.rainbow(255,255);
    public final LEDPattern rainbowScroll = rainbowPattern.scrollAtAbsoluteSpeed(FeetPerSecond.of(0.5), Feet.of(1.0/12.0));
    public  LEDPattern patternToApply = rainbowScroll;

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
     patternToApply = greenPattern;
    }

    public void purplelight () {
        patternToApply = purplePattern;
       }

    public void goldlight () {
        patternToApply = dimGoldPattern;
       }

    public void rainbowlight () {
      patternToApply = rainbowScroll;
    }










}
