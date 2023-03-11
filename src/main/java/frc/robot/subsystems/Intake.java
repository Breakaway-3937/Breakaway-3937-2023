// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor;
  private final AnalogInput uSSensor, bBSensor;
  private final GenericEntry usDistance, bBDistance;
  private static boolean cone, deadCone;

  public Intake() {
    intakeMotor = new WPI_TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    uSSensor = new AnalogInput(Constants.Intake.US_SENSOR_ID);
    uSSensor.resetAccumulator();
    bBSensor = new AnalogInput(Constants.Intake.BB_SENSOR_ID);
    bBSensor.resetAccumulator();
    usDistance = Shuffleboard.getTab("Intake").add("US Sensor", 0).withPosition(0, 0).getEntry();
    bBDistance = Shuffleboard.getTab("Intake").add("BB Sensor", 0).withPosition(1, 0).getEntry();
  }
  
  public void runIntake(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void spit(){
    intakeMotor.set(ControlMode.PercentOutput, -0.9);
  }
  
  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean intakeFull(){
    if(getConeCubeMode() || getDeadCone()){
      if(getDistance() < 0.35){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      if(bBSensor.getValue() > 4000){
        return true;
      }
      else{
        return false;
      }
    }
  }

  public void setCone(){
    cone = true;
    deadCone = false;
  }

  public void setCube(){
    cone = false;
    deadCone = false;
  }

  public void setDeadCone(){
    cone = false;
    deadCone = true;
  }

  public static boolean getConeCubeMode(){
    return cone;
  }

  public static boolean getDeadCone(){
    return deadCone;
  }

  public double getDistance(){
    return 0.342 - 0.291 * Math.log(uSSensor.getVoltage()) + 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    usDistance.setDouble(getDistance());
    bBDistance.setDouble(bBSensor.getValue());
  }
}
