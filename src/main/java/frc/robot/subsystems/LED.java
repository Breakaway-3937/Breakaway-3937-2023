// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class LED extends SubsystemBase {
    private final Intake s_Intake;
    private final CANdle candle;
    private final Timer timer, timer1;
    private boolean green, red, white, flag, flag1, flag2, flag3, flag4, flag5, flag6, cube, cone, bad = false; 

    public LED(Intake s_Intake) {
        candle = new CANdle(Constants.CANDLE_ID, "CANivore");
        timer = new Timer();
        timer1 = new Timer();
        this.s_Intake = s_Intake;
        timer.start();
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
        candle.setLEDs(0, 0, 0);
    }

    public void green(){
        red = false;
        white = false;
        green = true;
        bad = false;
    }

    public void red(){
        red = true;
        white = false;
        green = false;
        bad = false;
    }

    public void white(){
        red = false;
        white = true;
        green = false;
        bad = false;
    }

    public void yellow(){
        red = false;
        white = false;
        green = false;
        candle.setLEDs(255, 255, 0);
    }

    public void purple(){
        red = false;
        white = false;
        green = false;
        candle.setLEDs(136, 0, 209);
    }

    public void blue(){
        red = false;
        white = false;
        green = false;
        candle.setLEDs(0, 0, 254);
    }

    public void bad(){
        green = false;
        red = false;
        white = false;
        bad = true;
    }

    public void notBad(){
        bad = false;
    }

    public void setTrackingLEDsOff(){
        green = false;
        red = false;
        white = false;
        bad = false;
    }

    public void cone(){
        cube = false;
        cone = true;
    }

    public void cube(){
        cone = false;
        cube = true;
    }
    
    public void setOthersFalse(String color){
        if(color.equals("green")){
            flag4 = false;
            flag5 = false;
            flag6 = false;
        }
        else if(color.equals("red")){
            flag3 = false;
            flag5 = false;
            flag6 = false;
        }
        else if(color.equals("white")){
            flag4 = false;
            flag3 = false;
            flag6 = false;
        }
        else if(color.equals("blue")){
            flag4 = false;
            flag3 = false;
            flag5 = false;
        }
        else if(color.equals("all")){
            flag4 = false;
            flag3 = false;
            flag5 = false;
            flag6 = false;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(bad){
            for(int i = 0; i < 350; i++){
                if(timer.get() > 0.2 && i % 2 != 0 && !flag){
                    candle.setLEDs(12, 237, 54);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.2 && i % 2 == 0 && flag){
                    candle.setLEDs(179, 83, 97);
                    timer.reset();
                    flag = false;
                }
            }
        }
        else if(DriverStation.isDisabled()){
            //FIXME add led pattern
        }
        else if(!s_Intake.intakeFull()){
            setOthersFalse("all");
            if(cube){
                purple();
            }
            else if(cone){
                yellow();
            }
            flag1 = false;
        }
        else if(s_Intake.intakeFull() && !flag1){
            setOthersFalse("blue");
            if(!flag6){
                timer1.reset();
                timer1.start();
                flag2 = false;
                flag6 = true;
            }
            else if(timer1.get() > 1){
                flag2 = true;
            }
            for(int i = 0; i < 350; i++){
                if(!flag2){
                    if(timer.get() > 0.1 && i % 2 != 0 && !flag){
                        candle.setLEDs(0, 0, 254);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.1 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
                else{
                    blue();
                    flag1 = true;
                }
            }
        }
        else if(green){
            setOthersFalse("green");
            if(!flag3){
                timer1.reset();
                timer1.start();
                flag2 = false;
                flag3 = true;
            }
            else if(timer1.get() > 1){
                flag2 = true;
            }
            for(int i = 0; i < 350; i++){
                if(!flag2){
                    if(timer.get() > 0.1 && i % 2 != 0 && !flag){
                        candle.setLEDs(0, 255, 0);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.1 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
                else{
                    if(timer.get() > 0.2 && i % 2 != 0 && !flag){
                        candle.setLEDs(0, 255, 0);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.2 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
            }
        }
        else if(red){
            setOthersFalse("red");
            if(!flag4){
                timer1.reset();
                timer1.start();
                flag2 = false;
                flag4 = true;
            }
            else if(timer1.get() > 1){
                flag2 = true;
            }
            for(int i = 0; i < 350; i++){
                if(!flag2){
                    if(timer.get() > 0.1 && i % 2 != 0 && !flag){
                        candle.setLEDs(255, 0, 0);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.1 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
                else{
                    if(timer.get() > 0.2 && i % 2 != 0 && !flag){
                        candle.setLEDs(255, 0, 0);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.2 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
            }
        }
        else if(white){
            setOthersFalse("white");
            if(!flag5){
                timer1.reset();
                timer1.start();
                flag2 = false;
                flag5 = true;
            }
            else if(timer1.get() > 1){
                flag2 = true;
            }
            for(int i = 0; i < 350; i++){
                if(!flag2){
                    if(timer.get() > 0.1 && i % 2 != 0 && !flag){
                        candle.setLEDs(200, 180, 180);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.1 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
                else{
                    if(timer.get() > 0.2 && i % 2 != 0 && !flag){
                        candle.setLEDs(200, 180, 180);
                        timer.reset();
                        flag = true;
                    }
                    else if(timer.get() > 0.2 && i % 2 == 0 && flag){
                        candle.setLEDs(0, 0, 0);
                        timer.reset();
                        flag = false;
                    }
                }
            }
        }
    }
}
