package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.control.*;
import frc.lib.util.util.DrivetrainFeedforwardConstants;

/* Name All Variables in ALL_CAPS Format */

public final class Constants {
    public static final boolean FIELD_RELATIVE = true;
    public static final boolean OPEN_LOOP = true;
    public static final boolean COMP_BOT = false;
    public static final int CANDLE_ID = 15;
    public static final int PCM_ID = 16;
    public static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    public static class VisionConstants {
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(-0.15, -.10, 0.36),
                        new Rotation3d(0, 0, 0));
        public static final String CAMERA_NAME = "Global_Shutter_Camera";
        public static final double HIGH_LEFT_POST_X = -0.65;
        public static final double HIGH_LEFT_POST_Y = 0.560;
        public static final double HIGH_RIGHT_POST_X = -0.65;
        public static final double HIGH_RIGHT_POST_Y = -0.560;
        public static final double HIGH_CUBE_BOX_X = -0.65;
        public static final double HIGH_CUBE_BOX_Y = 0.0;
        public static final double LOW_CUBE_BOX_X = -0.22;
        public static final double LOW_CUBE_BOX_Y = 0.0;
        public static final double LOW_LEFT_POST_X = -0.220;
        public static final double LOW_LEFT_POST_Y = 0.560;
        public static final double LOW_RIGHT_POST_X = -0.220;
        public static final double LOW_RIGHT_POST_Y = -0.560;
        public static final double HIGH_DISTANCE = 0.85796;
        public static final double LOW_DISTANCE = 0.60166;
        public static final double HIGH_CUBE_BOX_DISTANCE = 0.65;
        public static final double LOW_CUBE_BOX_DISTANCE = 0.22;
        public static final double MAX_EXTEND_LENGTH = 1.22;
    }

    public static final class Controllers{
        public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
        public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
        public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
        public static final int XBOXCONTROLLER_A_BUTTON = 1;
        public static final int XBOXCONTROLLER_B_BUTTON = 2;
        public static final int XBOXCONTROLLER_X_BUTTON = 3;
        public static final int XBOXCONTROLLER_Y_BUTTON = 4;
        public static final int XBOXCONTROLLER_LB_BUTTON = 5;
        public static final int XBOXCONTROLLER_RB_BUTTON = 6;
        public static final int XBOXCONTROLLER_BACK_BUTTON = 7;
        public static final int XBOXCONTROLLER_START_BUTTON = 8;
        public static final double STICK_DEADBAND = 0.1;
        public static final int TRANSLATION_BUTTON = 1;
        public static final int ROTATION_BUTTON = 1;
        public static final int TRANSLATION_AXIS = 0;
        public static final int STRAFE_AXIS = 1;
        public static final int ROTATION_AXIS = 0;
    }

    public static final class DriveTrain {
        public static final int PIGEON_ID = 20;
        public static final boolean INVERT_GYRO = false;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.5461; 
        public static final double WHEEL_BASE = 0.5969; 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.7;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 12.0;
        public static final double ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_KS = (0.53906 / 12);
        public static final double DRIVE_KV = (2.2756 / 12);
        public static final double DRIVE_KA = (0.065383 / 12);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; //meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 0;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 21;
            public static final double ANGLE_OFFSET = 238.1835 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 19;
            public static final int ANGLE_MOTOR_ID = 18;
            public static final int CANCODER_ID = 22;
            public static final double ANGLE_OFFSET = 202.9 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CANCODER_ID = 23;
            public static final double ANGLE_OFFSET = 7.4 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CANCODER_ID = 24;
            public static final double ANGLE_OFFSET = 181.2304 + 180.0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(DRIVE_KV, DRIVE_KA, DRIVE_KS);

        public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(4.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(5.0), new CentripetalAccelerationConstraint(5.0)};  //FIXME

    }

    public static final class Intake{
        public static final int INTAKE_MOTOR_TOP = 7;
        public static final int INTAKE_MOTOR_BOTTOM = 6;
        public static final int WRIST_MOTOR_ID = 5; 
        public static final int SENSOR_ID = 0;
    }

    public static final class Climber{
        public static final int ID_1 = 12;
        public static final int ID_2 = 13;
    }

    public static final class Arm{
        public static final int SHOULDER_ID = 2;
        public static final int SHOULDER_2_ID = 3;
        public static final int EXTENSION_ID = 4;
        public static final int ROTATION_ID = 14;
        public static final SparkMaxAlternateEncoder.Type ALT_ENC_TYPE = SparkMaxAlternateEncoder.Type.kQuadrature;
        public static final int CPR = 8192;
    }

}
