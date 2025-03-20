// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PosElevate extends Command {
  /** Creates a new PosElevare. */
  Elevator m_elevator;
  Boolean stopCheck;
  Double resistance;
  Double inital;
  public PosElevate(Elevator m_elevator, Double resistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.resistance = resistance;
    this.m_elevator = m_elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    inital = m_elevator.getCurrentVoltage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inital < resistance){
      stopCheck = m_elevator.elevateUp(Constants.ElevatorConstants.ELEVATOR_SPEED, resistance);
    } else if (inital > resistance){
      stopCheck = m_elevator.elevateDown(Constants.ElevatorConstants.ELEVATOR_DOWN, resistance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.adjust(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
