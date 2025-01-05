// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDBase extends SubsystemBase {

    protected AddressableLED leds;
    protected AddressableLEDBuffer ledBuffer;
    protected int firstHue = 0;
    protected double lastChange;
    protected boolean on;

    public LEDBase(int ledPort, int ledLength) {
        leds = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    public void setSolidColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        leds.setData(ledBuffer);
    }

    public void setSolidColorDrivers(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        leds.setData(ledBuffer);
    }


    public void rainbowColorPattern() {
        int currentHue;

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            currentHue = (firstHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, currentHue, 255, 128);
        }

        firstHue = (firstHue + 3) % 180;
        leds.setData(ledBuffer);
    }

    public void blinkColorPattern(double interval, Color colorOne, Color colorTwo) {
        double timestamp = Timer.getFPGATimestamp();

        if (timestamp - lastChange > interval) {
            on = !on;
            lastChange = timestamp;
        }
        if (on) {
            setSolidColor(colorOne);
        } else {
            setSolidColor(colorTwo);
        }
    }

    public void waveColorPattern(int period, int numColors, Color[] colors) {
        double elapsedTime = Timer.getFPGATimestamp() % period;
        double progress = elapsedTime / period;
        int numLeds = ledBuffer.getLength();

        for (int i = 0; i < numLeds; i++) {
            double position = (double) i / (double) numLeds;
            double wavePosition = (position + progress) % 1.0;
            int colorIndex = (int) (wavePosition * numColors);

            Color currentColor = colors[colorIndex];
            ledBuffer.setLED(i, currentColor);
        }

        leds.setData(ledBuffer);
        updateLeds();
    }

    public void smoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
        double elapsedTime = Timer.getFPGATimestamp();
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            double position = ((double) i / ledBuffer.getLength()) + (elapsedTime * speed / period);
            double progress = position - (int) position;

            int startColorIndex = (int) (position % numColors);
            int endColorIndex = (startColorIndex + 1) % numColors;
            Color startColor = colors[startColorIndex];
            Color endColor = colors[endColorIndex];

            Color currentColor = new Color(
                    startColor.red + (endColor.red - startColor.red) * progress,
                    startColor.green + (endColor.green - startColor.green) * progress,
                    startColor.blue + (endColor.blue - startColor.blue) * progress
            );

            ledBuffer.setLED(i, currentColor);
            leds.setData(ledBuffer);
        }
    }

    public void updateLeds() {
        leds.setData(ledBuffer);
    }
}