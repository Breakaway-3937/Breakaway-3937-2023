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
    private int r, g, b, num; 
    private int i = 8;
    private int count = 1;

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
        configAll.brightnessScalar = 0.5;
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

    public void setTrackingLEDsOff(boolean bad){
        green = false;
        red = false;
        white = false;
        if(DriverStation.isDisabled()){
            this.bad = bad;
        }
        else{
            this.bad = false;
        }
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
            if(timer.get() > 0.2 && !flag){
                candle.setLEDs(12, 237, 54);
                timer.reset();
                flag = true;
            }
            else if(timer.get() > 0.2 && flag){
                candle.setLEDs(179, 83, 97);
                timer.reset();
                flag = false;
            }
        }
        else if(DriverStation.isDisabled()){
            if(i > 16){
                count = 8;
            }
            if(i < 16){
                count = i - 8;
            }
            if(i >= 49){
                if(timer.get() > 0.01){
                    candle.setLEDs(0, 0, 0, 0, i - 2, 1);
                    candle.setLEDs(0, 0, 0, 0, i + 49 - 2, 1);
                    timer.reset();
                    i++;
                    if(i == 60){
                        i = 8;
                        if(num == 6){
                            num = 0;
                        }
                        else{
                            num++;
                        }
                    }
                }
            }
            else if(i < 49){
                if(timer.get() > 0.01 && !flag){
                    candle.setLEDs(r, g, b, 0, i, count);
                    candle.setLEDs(r, g, b, 0, i + 49, count);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.01 && flag){
                    candle.setLEDs(0, 0, 0, 0, i - 2, 1);
                    candle.setLEDs(0, 0, 0, 0, i + 49 - 2, 1);
                    timer.reset();
                    flag = false;
                    i++;
                }
            }

            switch(num){
                case 0:
                r = 255;
                g = 0;
                b = 0;
                break;

                case 1:
                r = 0;
                g = 0;
                b = 255;
                break;

                case 2:
                r = 0;
                g = 255;
                b = 0;
                break;

                case 3:
                r = 255;
                g = 226;
                b = 2;
                break;

                case 4:
                r = 255;
                g = 0;
                b = 255;
                break;

                case 5:
                r = 0;
                g = 244;
                b = 255;
                break;

                case 6:
                r = 220;
                g = 88;
                b = 42;
                break;
            }
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
            if(!flag2){
                if(timer.get() > 0.1 && !flag){
                    candle.setLEDs(0, 0, 254);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.1 && flag){
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
            if(!flag2){
                if(timer.get() > 0.1 && !flag){
                    candle.setLEDs(0, 255, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.1 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
            else{
                if(timer.get() > 0.2 && !flag){
                    candle.setLEDs(0, 255, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.2 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
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
            if(!flag2){
                if(timer.get() > 0.1 && !flag){
                    candle.setLEDs(255, 0, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.1 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
            else{
                if(timer.get() > 0.2 && !flag){
                    candle.setLEDs(255, 0, 0);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.2 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
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
            if(!flag2){
                if(timer.get() > 0.1 && !flag){
                    candle.setLEDs(200, 180, 180);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.1 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
            else{
                if(timer.get() > 0.2 && !flag){
                    candle.setLEDs(200, 180, 180);
                    timer.reset();
                    flag = true;
                }
                else if(timer.get() > 0.2 && flag){
                    candle.setLEDs(0, 0, 0);
                    timer.reset();
                    flag = false;
                }
            }
        }
    }
}
