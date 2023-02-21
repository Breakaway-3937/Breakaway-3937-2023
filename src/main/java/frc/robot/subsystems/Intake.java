// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final AnalogInput uSSensor, lightSensor;
  private final GenericEntry usDistance, lightDistance;
  private static boolean cone;

  public Intake() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    uSSensor = new AnalogInput(Constants.Intake.US_SENSOR_ID);
    uSSensor.resetAccumulator();
    lightSensor = new AnalogInput(Constants.Intake.LIGHT_SENSOR_ID);
    lightSensor.resetAccumulator();
    usDistance = Shuffleboard.getTab("Intake").add("US Sensor", 0).withPosition(0, 0).getEntry();
    lightDistance = Shuffleboard.getTab("Intake").add("Light Sensor", 0).withPosition(1, 0).getEntry();
  }
  
  public void runIntake(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void spit(){
    intakeMotor.set(ControlMode.PercentOutput, -0.8);
  }
  
  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean intakeFull(){
    if(getConeCubeMode()){
      if(getDistance() < 0.35){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      if(lightSensor.getValue() > 4000){
        return true;
      }
      else{
        return false;
      }
    }
  }

  public void setCone(){
    cone = true;
  }

  public void setCube(){
    cone = false;
  }

  public static boolean getConeCubeMode(){
    return cone;
  }

  public double getDistance(){
    return 0.342 - 0.291 * Math.log(uSSensor.getVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    usDistance.setDouble(getDistance());
    lightDistance.setDouble(lightSensor.getValue());
  }
}
