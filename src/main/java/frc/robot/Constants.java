package frc.robot;

public class Constants {
    public class SwerveConstants {
        public static final double SlowSpeed = 0.1; // Speed multiplier for slow mode
        public static final double deadband = 0.1; // Controller deadband
        public static final double speedMultiplier = 0.8; // Scaler for speed in all modes
        public static final double SlowAngle = 0.1; // Angle multiplier for slow mode
        public static final double SlewLimit_Drive = 0.5;
        public static final double SlewLimit_Turn = 0.5;
        public static final Integer FRONT_RIGHT = 6;
        public static final Integer FRONT_LEFT = 12;
        public static final Integer BACK_RIGHT = 3;
        public static final Integer BACK_LEFT = 9;
    }

    public class ManipulatorConstants {
        public static final double MANIPULATE_SPEED  = 0.15;
        public static final int MANIPULATORLEFT = 15;
        public static final int MANIPULATORRIGHT = 16;
    }

    public class ClimberConstants {
        public static final double CLIMB_SPEED  = 0.25;
        public static final int CLIMBER = 17;
    }

    public class ElevatorConstants {
        public static final int ELEVATOR = 19;
        public static final int ELEVATOR2 = 20;
        public static final int ELEVATOR_ENCODER = 20;
        public static final double ELEVATORSPEED = .3;
        public static final int L1ANGLE = 0;
        public static final int L2ANGLE = 0;
        public static final int L3ANGLE = 0;
        public static final int L4ANGLE = 0;
        public static final int BARGEANGLE = 0;            
    }
}
