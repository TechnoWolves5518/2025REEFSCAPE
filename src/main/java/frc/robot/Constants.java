package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.units.Units;


public class Constants {
    public static class VisionC {
        public static final String frontLeftCamera = "AprilCamFL";
        public static final String frontRightCamera = "AprilCamFR";
        public static final String rearLeftCamera = "AprilCamRL";
        public static final String rearRightCamera = "AprilCamRR";
    }
    public static class VisionSystem {
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.Meters.convertFrom(Units.Millimeters.convertFrom(15, Units.Inches) - 100, Units.Millimeters), Units.Meters.convertFrom((Units.Millimeters.convertFrom(15, Units.Inches) - 90), Units.Millimeters), Units.Meters.convertFrom(177, Units.Millimeters)), new Rotation3d(0, 0, Units.Radians.convertFrom(-33.3, Units.Degrees)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
    public class SwerveConstants {
        public static RobotConfig config;
        static {
            RobotConfig tempConfig = null;
            try {
                tempConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
            config = tempConfig;
        }
        public static final double SlowSpeed = 0.4; // Speed multiplier for slow mode
        public static final double deadband = 0.1; // Controller deadband
        public static final double speedMultiplier = 0.8; // Scaler for speed in all modes
        public static final double SlowAngle = 0.4; // TIME multiplier for slow mode
        public static final double SlewLimit_Drive = 2;
        public static final double SlewLimit_Turn = 2;
        public static final Integer FRONT_RIGHT = 6;
        public static final Integer FRONT_LEFT = 12;
        public static final Integer BACK_RIGHT = 3;
        public static final Integer BACK_LEFT = 9;
        public static final Double DEADBAND = 0.1;
    }

    
    public class ManipulatorConstants {
        public static final double MANIPULATE_SPEED  = 0.25;
        public static final int MANIPULATORLEFT = 15;
        public static final int MANIPULATORRIGHT = 16;
    }
    
    public class ClimberConstants {
        public static final double CLIMB_SPEED  = .6;
        public static final double REVERSE_CLIMB_SPEED = -0.125;
        public static final int CLIMBER = 17;
        public static final Integer SERVO_NUMBER = 1;
        public static final int SERVO_ON = 40;
        public static final int SERVO_OFF = 0;
    }

    public class ElevatorConstants {
        public static final int ELEVATOR = 19;
        public static final int ELEVATOR2 = 20;
        //public static final int ELEVATOR_ENCODER = 20;
        public static final double ELEVATOR_SPEED = -.8;
        public static final double ELEVATOR_DOWN = .1;
        public static final double SPROCKET_RADIUS = 0.606;
        public static final double SHAFT_SPEED = ((3375.345 * .8)/60)/5.95; // Calculates the rps of the elevator output shaft
        public static final double SHAFT_SPEED_DOWN = ((3375.345 * .1)/60)/5.95; // Calculates the rps of the elevator output shaft
        public static final double ELEVATOR_RATE = (2*Math.PI*SPROCKET_RADIUS) * SHAFT_SPEED; // Calculates delta height per rotation of elevator output shaft
        public static final double ELEVATOR_RATE_DOWN = (2*Math.PI*SPROCKET_RADIUS) * SHAFT_SPEED_DOWN; // Calculates delta height per rotation of elevator output shaft
        // units: inches
        public static final int L1_HEIGHT = 20;
        public static final int L2_HEIGHT = 30;
        public static final int L3_HEIGHT = 50;
        public static final int L4_HEIGHT = 75;
    }



    
}
