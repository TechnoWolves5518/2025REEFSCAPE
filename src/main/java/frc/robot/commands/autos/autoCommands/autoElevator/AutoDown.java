// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.autoCommands.autoElevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDown extends Command {
  /** Creates a new AutoDown. */
  Elevator m_elevator;
  int height;
  int timer;
  int time;
  boolean stopCheck;
  public AutoDown(Elevator m_elevator, int height) {
    this.m_elevator = m_elevator;
    this.height = height;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    timer = 0;
    m_elevator.adjust(Constants.ElevatorConstants.ELEVATOR_DOWN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = (int)(50 * (Math.round(height/Constants.ElevatorConstants.ELEVATOR_RATE_DOWN)));

    if (timer < time) {
      timer++;
    } else {
      stopCheck = true;
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
