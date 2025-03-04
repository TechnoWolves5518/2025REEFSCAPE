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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.SwerveConstants;

public class RobotContainer {
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

    SlewRateLimiter driverLeftXLimiter = new SlewRateLimiter(.5);
    SlewRateLimiter driverLeftYLimiter = new SlewRateLimiter(.5);
    SlewRateLimiter driverRightXLimiter = new SlewRateLimiter(.5);
    SlewRateLimiter driverRightYLimiter = new SlewRateLimiter(.5);
      
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

    private double speedMultiplier = 1;
    private double angleMultiplier = 1;
 



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

        driverXbox.start().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.back().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        ));
        
        // Toggle slow mode when left bumper is pressed.
        driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {slowMode = !slowMode;
          SmartDashboard.putBoolean("Slow", slowMode);
          if (slowMode) {
            speedMultiplier = SwerveConstants.SlowSpeed;
            angleMultiplier = SwerveConstants.SlowAngle;
        }
        else {
            speedMultiplier = 1;
            angleMultiplier = 1;
        }
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-driverRightXLimited() * MaxAngularRate * angleMultiplier) // Drive counterclockwise with negative X (left)
            )
        );
      }));
        
        // Drive robot oriented while right bumper is held.
        driverXbox.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            driveRobot.withVelocityX(-driverXbox.getLeftY() * TotalMaxSpeed * speedMultiplier)
                .withVelocityY(-driverXbox.getLeftX() * TotalMaxSpeed * speedMultiplier)
                .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate * speedMultiplier)
        ));

        // reset the field-centric heading on left bumper press
        driverXbox.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
