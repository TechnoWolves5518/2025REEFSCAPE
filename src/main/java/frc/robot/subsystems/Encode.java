// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Encode extends SubsystemBase {
  /** Creates a new Encode. */
  static CANcoder frontRight;
  static CANcoder frontLeft;
  static CANcoder backRight;
  static CANcoder backLeft;
  public Encode() {
    frontRight = new CANcoder(Constants.FRONT_RIGHT);
    frontLeft = new CANcoder(Constants.FRONT_LEFT);
    backRight = new CANcoder(Constants.BACK_RIGHT);
    backLeft = new CANcoder(Constants.BACK_LEFT);
  }
  public void read() {
    SmartDashboard.putNumber("Front Right: ", frontRight.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Front Left: ", frontLeft.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Back Right: ", backRight.getAbsolutePosition().getValue().in(Units.Degrees));
    SmartDashboard.putNumber("Back Left: ", backLeft.getAbsolutePosition().getValue().in(Units.Degrees));
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
