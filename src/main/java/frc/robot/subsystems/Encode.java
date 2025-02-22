// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Encode extends SubsystemBase {
  /** Creates a new Encode. */
  static TalonFX frontRight;
  static TalonFX frontLeft;
  static TalonFX backRight;
  static TalonFX backLeft;
  public Encode() {
    frontRight = new TalonFX(Constants.FRONT_RIGHT);
    frontLeft = new TalonFX(Constants.FRONT_LEFT);
    backRight = new TalonFX(Constants.BACK_RIGHT);
    backLeft = new TalonFX(Constants.BACK_LEFT);
  }

  public void read() {
    SmartDashboard.putNumber("Front Right: ", frontRight.get());
    SmartDashboard.putNumber("Front Left: ", frontLeft.get());
    SmartDashboard.putNumber("Back Right: ", backRight.get());
    SmartDashboard.putNumber("Back Left: ", backLeft.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
