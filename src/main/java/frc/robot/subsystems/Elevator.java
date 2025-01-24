// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  static CANVenom elevateMotor;
  static CANcoder spinReader;

  public Elevator() {
    elevateMotor = new CANVenom(Constants.ELEVATOR);
    spinReader = new CANcoder(Constants.ELEVATOR_ENCODER);
  }

  public void elevate(double speed, double angle){
    double currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
     if (currentAngle < angle){
        while (currentAngle < angle){
          elevateMotor.set(speed);
          currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
        }
      }
      else if (currentAngle > angle){
        while (currentAngle > angle){
        elevateMotor.set(-speed);
        currentAngle = spinReader.getPosition().getValue().in(Units.Degrees);
      }
    }
     
  }

  public void hold(){
    elevateMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  public void elevatorRead(){
    SmartDashboard.putNumber("Elevator RAW angle", spinReader.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Elevator angle", spinReader.getPosition().getValue().in(Units.Degrees));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
