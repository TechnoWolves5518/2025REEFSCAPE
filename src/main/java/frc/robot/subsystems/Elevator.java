// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  static TalonSRX elevateMotor1;
  static TalonSRX elevateMotor2;
  static CANcoder spinReader;

  public Elevator() {
    elevateMotor1 = new TalonSRX(Constants.ElevatorConstants.ELEVATOR);
    elevateMotor2 = new TalonSRX(Constants.ElevatorConstants.ELEVATOR2);
    spinReader = new CANcoder(Constants.ElevatorConstants.ELEVATOR_ENCODER);
    elevateMotor2.follow(elevateMotor1);
    elevateMotor1.setNeutralMode(NeutralMode.Brake);
    elevateMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void elevate(double speed, double angle){
    double currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
     if (currentAngle < angle){
        while (currentAngle < angle){
          elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed * (currentAngle/10));
          currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
        }
      }
      else if (currentAngle > angle){
        while (currentAngle > angle){
        elevateMotor1.set(TalonSRXControlMode.PercentOutput,-speed * (currentAngle/10));
        currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
      }
    }
     
  }



  public void elevatorRead(){
    SmartDashboard.putNumber("Elevator RAW angle", spinReader.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Elevator angle", spinReader.getPosition().getValue().in(Units.Degrees));
  }

  public void adjust(double speed){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void hold(){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, -.25);
  }

  public void release(double speed){
    elevateMotor1.set(TalonSRXControlMode.PercentOutput, speed/.75);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
