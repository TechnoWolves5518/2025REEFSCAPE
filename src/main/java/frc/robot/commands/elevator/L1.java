// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1 extends Command {
  /** Creates a new L1. */
  Elevator m_elevator;
  boolean stopCheck;
  int time;
  int currentTime;
  public L1(Elevator m_elevator) {
    this.m_elevator = m_elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    time = (int)(50 * (Math.round(30/Constants.ElevatorConstants.ELEVATOR_RATE))) / 2;
    System.out.println("Time: " + time);
    currentTime = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (currentTime <= time){
      m_elevator.adjust(Constants.ElevatorConstants.ELEVATOR_SPEED);
      currentTime++;
    }else{
      stopCheck = true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
