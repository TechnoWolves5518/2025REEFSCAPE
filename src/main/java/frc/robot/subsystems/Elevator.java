// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  static TalonSRX elevateMotor1;
  static TalonSRX elevateMotor2;
  static CANcoder spinReader;
  static AnalogInput posReader;

  public Elevator() {
    elevateMotor1 = new TalonSRX(Constants.ElevatorConstants.ELEVATOR);
    elevateMotor2 = new TalonSRX(Constants.ElevatorConstants.ELEVATOR2);

    posReader = new AnalogInput(0);
    
    elevateMotor2.follow(elevateMotor1);
    elevateMotor1.setNeutralMode(NeutralMode.Brake);
    elevateMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public boolean elevate(Double speed, Double resistance){
    double currentResistance = posReader.getVoltage();
    if ((Math.abs(currentResistance - resistance) > .002)){
      //elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed);
      elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed);
      return false;
    }
    else {
      return true;
    }
  }

  
 


  public Double ProportionalSpeed(double target){
    return (Math.max(-1 ,Math.min(1, 0.35 + 0.25*(target-getCurrentVoltage()))));
  }


  public Double getCurrentVoltage() {
    return posReader.getVoltage();
  }


  public void elevatorRead(){
    SmartDashboard.putNumber("Elevator Pos: ", getCurrentVoltage());
    SmartDashboard.updateValues();
  }

  public void adjust(double speed){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void hold(){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, Constants.ElevatorConstants.FEED_FOWARD);
    System.out.println("Current Resistance: " + getCurrentVoltage());
  }


  public void release(double speed){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed/.75);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
