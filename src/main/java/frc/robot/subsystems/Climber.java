// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  static TalonSRX climber;
  Servo servo;

  public Climber() {
    climber = new TalonSRX(Constants.ClimberConstants.CLIMBER);
    servo = new Servo(Constants.ClimberConstants.SERVO_NUMBER);
  }

  public void setServo(int value) {
    servo.setAngle(value);
  }

  public void climb(double speed, int value) {
    servo.setAngle(value);
    for (int i = 0; i < 10; i++) { // delay

    }
    climber.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
