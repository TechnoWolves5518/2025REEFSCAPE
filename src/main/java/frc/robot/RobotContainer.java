// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.Manipulate;
import frc.robot.commands.ReverseClimb;
import frc.robot.commands.ReverseManipulate;
import frc.robot.commands.elevator.Down;
import frc.robot.commands.elevator.Hold;
import frc.robot.commands.elevator.L1;
import frc.robot.commands.elevator.L2;
import frc.robot.commands.elevator.L3;
import frc.robot.commands.elevator.L4;
import frc.robot.commands.elevator.ReturnZero;
import frc.robot.commands.elevator.Up;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
public class RobotContainer {
  private final Elevator elevator = new Elevator();
  private final Climber climber = new Climber();
  private final Manipulator manipulate = new Manipulator();
  private SendableChooser<Command> autoChooser;
  private double speedMultiplier = 1;
  private double angleMultiplier = 1;

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
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandJoystick driverJoystick = new CommandJoystick(2);
    private final CommandXboxController schmoXbox = new CommandXboxController(1);

    SlewRateLimiter driverLeftXLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter driverLeftYLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter driverRightXLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Turn);
    SlewRateLimiter driverRightYLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Turn);
    SlewRateLimiter joystickXLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter joystickYLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Drive);
    SlewRateLimiter joystickZLimiter = new SlewRateLimiter(Constants.SwerveConstants.SlewLimit_Turn);
      
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
    double joystickXLimited() {
      return joystickXLimiter.calculate(driverJoystick.getX() * -driverJoystick.getZ());
    }
    double joystickYLimited() {
      return joystickYLimiter.calculate(driverJoystick.getY() * -driverJoystick.getZ());
    }
    double joystickTLimited() {
      driverJoystick.setTwistChannel(5);
      return joystickZLimiter.calculate(driverJoystick.getTwist() * -driverJoystick.getZ());
    }

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }
 

    public void configureJoysticks() {
      if(driverJoystick.isConnected()) {
        drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
          drive.withVelocityX(-joystickYLimited() * TotalMaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-joystickXLimited() * TotalMaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-joystickTLimited() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
          )
      );
      }
      else{
        drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
          drive.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-driverRightXLimited() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
          )
      );}
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        configureJoysticks();
        // Schmo commands:
        schmoXbox.pov(0).whileTrue(new Up(elevator)).whileFalse(new Hold(elevator));
        schmoXbox.pov(180).whileTrue(new Down(elevator)).whileFalse(new Hold(elevator));
        //schmoXbox.pov(0).and(schmoXbox.pov(180));
        schmoXbox.leftTrigger().whileTrue(new Manipulate(manipulate));
        schmoXbox.rightTrigger().whileTrue(new ReverseManipulate(manipulate));
        schmoXbox.leftBumper().whileTrue(new ReverseClimb(climber));
        schmoXbox.rightBumper().whileTrue(new Climb(climber));
        schmoXbox.a().onTrue(new  L1(elevator));
        schmoXbox.b().onTrue(new L2(elevator));
        schmoXbox.x().onTrue(new L3(elevator));
        schmoXbox.y().onTrue(new L4(elevator));
        schmoXbox.back().onTrue(new ReturnZero(elevator));
        driverXbox.start().whileTrue(drivetrain.applyRequest(() -> brake));

        // When the left trigger on driver xbox controller is pressed, drive in slow mode.
        driverXbox.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverRightXLimited() * MaxAngularRate * Constants.SwerveConstants.SlowAngle) // Drive counterclockwise with negative X (left)
            .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
        ));

        // When a bumper on the driver xbox controller is pressed 
        driverXbox.leftBumper().or(driverXbox.rightBumper()).whileTrue(drivetrain.applyRequest(() ->
        driveRobot.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed * Constants.SwerveConstants.SlowSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverRightXLimited() * MaxAngularRate * Constants.SwerveConstants.SlowAngle) // Drive counterclockwise with negative X (left)
            .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
        ));

        // When the right trigger on the driver xbox controller is pressed, drive robot oriented.
        driverXbox.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
        driveRobot.withVelocityX(-driverLeftYLimited() * TotalMaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driverLeftXLimited() * TotalMaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverRightXLimited() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
        ));

        // Switch to robot oriented mode when the trigger on the joystick is pressed.
        driverJoystick.button(1).whileTrue(drivetrain.applyRequest(() ->
        driveRobot.withVelocityX(-joystickYLimited() * TotalMaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystickXLimited() * TotalMaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystickTLimited() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            .withDeadband(Constants.SwerveConstants.deadband) // Add a deadband
        ));

        // Brake when button L1 on the joystick is pressed.
        driverJoystick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field centric orientation when button L3 on the joystick is pressed
        driverJoystick.button(4).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // reset the field-centric heading on y button press
        driverXbox.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        boolean isComp = false;

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
          (stream) -> isComp
          ? stream.filter(auto -> auto.getName().contains("comp"))
          : stream
        );

        NamedCommands.registerCommand("L1", new L1(elevator));
        NamedCommands.registerCommand("L2", new L2(elevator));
        NamedCommands.registerCommand("L3", new L3(elevator));
        NamedCommands.registerCommand("L4", new L4(elevator));
        NamedCommands.registerCommand("ReturnZero", new ReturnZero(elevator));
        NamedCommands.registerCommand("Climb", new Climb(climber));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
      return new PathPlannerAuto("Example Auto");
    }
}
