// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorV2 extends SubsystemBase {
  static TalonSRX elevateMotor;
  static CANcoder spinReader;
  static double dt;
  /** Creates a new ElevatorV2. */
  public ElevatorV2() {
    elevateMotor = new TalonSRX(Constants.ELEVATOR);
    spinReader = new CANcoder(Constants.ELEVATOR_ENCODER);
    dt = 0.02;


  }

  public void elevate(double speed){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
