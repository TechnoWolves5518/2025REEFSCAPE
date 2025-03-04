// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;


import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  static TalonSRX elevateMotor;
  static CANcoder spinReader;
  static TalonSRX othermotor;

  public Elevator() {
    elevateMotor = new TalonSRX(Constants.ELEVATOR);
    othermotor = new TalonSRX(20);
    spinReader = new CANcoder(Constants.ELEVATOR_ENCODER);
    elevateMotor.setNeutralMode(NeutralMode.Brake);
    othermotor.setNeutralMode(NeutralMode.Brake);

    
  }

  public void elevate(double speed, double angle){
    double currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
     if (currentAngle < angle){
        while (currentAngle < angle){
          elevateMotor.set(TalonSRXControlMode.PercentOutput, speed * (currentAngle/10));
          currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
        }
      }
      else if (currentAngle > angle){
        while (currentAngle > angle){
        elevateMotor.set(TalonSRXControlMode.PercentOutput,-speed * (currentAngle/10));
        currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
      }
    }
     
  }



  public void elevatorRead(){
    SmartDashboard.putNumber("Elevator RAW angle", spinReader.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Elevator angle", spinReader.getPosition().getValue().in(Units.Degrees));
  }

  public void adjust(double speed){
    elevateMotor.set(TalonSRXControlMode.PercentOutput, speed);
    othermotor.follow(elevateMotor);
  }

  public void release(double speed){
    elevateMotor.set(TalonSRXControlMode.PercentOutput, speed/.75);
    elevateMotor.follow(elevateMotor);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
