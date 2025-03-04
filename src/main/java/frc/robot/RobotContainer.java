// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.commands.elevator.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final Elevator elevator = new Elevator();
    private final Encode encode = new Encode();
    private final Climber climb = new Climber();
    private final Manipulator manipulate = new Manipulator();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double TotalMaxSpeed = MaxSpeed * SwerveConstants.speedMultiplier; // Total maximum speed of robot
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * SwerveConstants.deadband).withRotationalDeadband(MaxAngularRate * SwerveConstants.deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * SwerveConstants.deadband).withRotationalDeadband(MaxAngularRate * SwerveConstants.deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private boolean fieldCentric = false;
    private boolean slowMode = false;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController schmoXbox = new CommandXboxController(1);

    SlewRateLimiter driverLeftXLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter driverLeftYLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter driverRightXLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Turn);
    SlewRateLimiter driverRightYLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Turn);
      
    // Functions to get the slew rate limited values of the joysticks
    double driverLeftXLimited() {
      return driverLeftXLimiter.calculate(driverXbox.getLeftX());
    }
    double driverLeftYLimited() {
      return driverLeftYLimiter.calculate(driverXbox.getLeftY());
    }
    double driverRightXLimited() {
      return driverRightXLimiter.calculate(driverXbox.getRightX());
    }
    double driverRightYLimited() {
      return driverRightYLimiter.calculate(driverXbox.getRightY());
    }

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }
 



    private void configureBindings() {
      SmartDashboard.putBoolean("Slow", slowMode);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverRightXLimited() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Schmo commands:
        schmoXbox.pov(180).whileTrue(new Up(elevator));
        schmoXbox.pov(0).whileTrue(new Down(elevator));
        schmoXbox.pov(0).and(schmoXbox.pov(180)).whileFalse(new Brake(elevator));
        schmoXbox.leftTrigger().whileTrue(new ReverseManipulate(manipulate));
        schmoXbox.rightTrigger().whileTrue(new ReverseManipulate(manipulate));
        schmoXbox.a().onTrue(new  L1(elevator));
        schmoXbox.b().onTrue(new L2(elevator));
        schmoXbox.x().onTrue(new L3(elevator));
        schmoXbox.y().onTrue(new L4(elevator));
        schmoXbox.back().onTrue(new ReturnZero(elevator));
        schmoXbox.start().onTrue(new Barge(elevator));
        

        driverXbox.start().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.back().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        ));
        
        // Toggle slow mode when left bumper is pressed.
        driverXbox.leftBumper().and(() -> !driverXbox.rightBumper().getAsBoolean()).whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverRightXLimited() * MaxAngularRate * Constants.SwerveConstants.SlowAngle) // Drive counterclockwise with negative X (left)
    ));
        
        // Drive robot oriented while right bumper is held.
        driverXbox.rightBumper().and(() -> !driverXbox.leftBumper().getAsBoolean()).whileTrue(drivetrain.applyRequest(() ->
            driveRobot.withVelocityX(-driverXbox.getLeftY() * TotalMaxSpeed)
                .withVelocityY(-driverXbox.getLeftX() * TotalMaxSpeed)
                .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate)
        ));

        // Drive robot oriented and slow when the right and left bumpers are held
        driverXbox.rightBumper().and(driverXbox.leftBumper()).whileTrue(drivetrain.applyRequest(() ->
            driveRobot.withVelocityX(-driverXbox.getLeftY() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed)
                .withVelocityY(-driverXbox.getLeftX() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed)
                .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate * Constants.SwerveConstants.SlowAngle)
        ));

        // reset the field-centric heading on left bumper press
        driverXbox.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
