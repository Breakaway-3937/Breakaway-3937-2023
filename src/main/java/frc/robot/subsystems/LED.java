// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(Constants.CANDLE_ID, "CANivore");
    private final Timer timer = new Timer();
    private boolean green, red, blinkYellow, purple, solidYellow, blue, flag = false; 

    public LED() {
        timer.start();
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
        candle.setLEDs(0, 0, 0, 0, 0, 400);
    }


    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat(){
        return candle.getBusVoltage(); 
    }
    public double get5V(){ 
        return candle.get5VRailVoltage(); 
    }
    public double getCurrent(){ 
        return candle.getCurrent(); 
    }
    public double getTemperature(){
        return candle.getTemperature(); 
    }
    public void configBrightness(double percent){ 
        candle.configBrightnessScalar(percent, 0); 
    }
    public void configLos(boolean disableWhenLos){ 
        candle.configLOSBehavior(disableWhenLos, 0); 
    }
    public void configLedType(LEDStripType type){ 
        candle.configLEDType(type, 0); 
    }
    public void configStatusLedBehavior(boolean offWhenActive){ 
        candle.configStatusLedState(offWhenActive, 0); 
    }

    public void green(){
        red = false;
        blue = false;
        purple = false;
        blinkYellow = false;
        solidYellow = false;
        green = true;
    }

    public void red(){
        red = true;
        blue = false;
        purple = false;
        blinkYellow = false;
        solidYellow = false;
        green = false;
    }

    public void blinkYellow(){
        red = false;
        blue = false;
        purple = false;
        blinkYellow = true;
        solidYellow = false;
        green = false;
    }

    public void solidYellow(){
        red = false;
        blue = false;
        purple = false;
        blinkYellow = false;
        solidYellow = true;
        green = false;
        candle.setLEDs(255, 255, 0);
    }

    public void purple(){
        red = false;
        blue = false;
        purple = true;
        blinkYellow = false;
        solidYellow = false;
        green = false;
        candle.setLEDs(136, 0, 209);
    }

    public void blue(){
        red = false;
        blue = true;
        purple = false;
        blinkYellow = false;
        solidYellow = false;
        green = false;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(green){
            for(int i = 0; i < 350; i++){
                if(timer.get() > 0.25 && i % 2 != 0 && !flag){
                    candle.setLEDs(20, 255, 5);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.25 && i % 2 == 0 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
        }
        else if(red){
            for(int i = 0; i < 350; i++){
                if(timer.get() > 0.25 && i % 2 != 0 && !flag){
                    candle.setLEDs(255, 0, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.25 && i % 2 == 0 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
        }
        else if(blinkYellow){
            for(int i = 0; i < 350; i++){
                if(timer.get() > 0.25 && i % 2 != 0 && !flag){
                    candle.setLEDs(255, 255, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.25 && i % 2 == 0 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
        }
        else if(blue){
            for(int i = 0; i < 350; i++){
                if(timer.get() > 0.25 && i % 2 != 0 && !flag){
                    candle.setLEDs(0, 20, 255);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.25 && i % 2 == 0 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
        }
    }
}
