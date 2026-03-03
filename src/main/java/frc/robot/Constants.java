// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
  }

  public class drivetrainConstants {
    public static final int frontLeftDriveMotorId = 0;
    public static final int frontLeftSteerMotorId = 1;  
    public static final int frontLeftDriveEncoderId = 2;
    public static final int frontRightDriveMotorId = 3;
    public static final int frontRightSteerMotorId = 4;  
    public static final int frontRightDriveEncoderId = 5;
    public static final int rearLeftDriveMotorId = 6;
    public static final int rearLeftSteerMotorId = 7;  
    public static final int rearLeftDriveEncoderId = 8;
    public static final int rearRightDriveMotorId = 9;
    public static final int rearRightSteerMotorId = 10;  
    public static final int rearRightDriveEncoderId = 11;
  }

  public class IndexerConstants {
    public static final int indexerMotorId = 12;
    public static final int indexerWallMotorId = 13; 
  }

  public class KickerConstants {
    public static final int kickerMotorId = 14; 
  }

  public class ShooterConstants {
    public static final int leadShooterMotorId = 15; //15
    public static final int followerShooterMotorId = 16; //16
    public static final double shooterPosition1VelocityFPS = 55;
    public static final double shooterPosition2VelocityFPS = 63;
    public static final double shooterPosition3VelocityFPS = 52;
    
    public static final int hoodMotorId = 17; //17
    public static final double hoodPosition1Angle = 22;
    public static final double hoodPosition2Angle = 36.3;
    public static final double hoodPosition3Angle = 25.17;

    public static final int turretAimMotorId = 18; //18
  }

  public class IntakeConstants{
    public static final int intakeRollersMotorId = 19; //19
       public static final int intakePivotMotorId = 20; //20
    public static final int intakePivotCANcoderId = 21;

    public static final double intakePivotStartAngleDegrees = 0;
    public static final double  intakePivotRetractAngleDegrees = 0.1;
    public static final double  intakePivotOutAngleDegrees = 136;
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camFrontLeft";
    public static String camera1Name = "camFrontRight";
    public static String camera2Name = "camRearLeft";
    public static String camera3Name = "camRearRight";


    public static double camFrontLeftX = Units.inchesToMeters(-10.466);
    public static double camFrontLeftY = Units.inchesToMeters(12.3);
    public static double camFrontLeftZ = Units.inchesToMeters(20.375);
    public static double camFrontLeftPitch = Units.degreesToRadians(-15);
    public static double camFrontLeftYaw = Units.degreesToRadians(10);
    
    public static double camFrontRightX = Units.inchesToMeters(-10.466);
    public static double camFrontRightY = Units.inchesToMeters(-12.3);
    public static double camFrontRightZ = Units.inchesToMeters(20.375);
    public static double camFrontRightPitch = Units.degreesToRadians(-15);
    public static double camFrontRightYaw = Units.degreesToRadians(-10);

    public static double camRearLeftX = Units.inchesToMeters(-12.53);
    public static double camRearLeftY = Units.inchesToMeters(12.3);
    public static double camRearLeftZ = Units.inchesToMeters(20.375);
    public static double camRearLeftPitch = Units.degreesToRadians(-15);
    public static double camRearLeftYaw = Units.degreesToRadians(170);

    public static double camRearRightX = Units.inchesToMeters(-12.53);
    public static double camRearRightY = Units.inchesToMeters(-12.3);
    public static double camRearRightZ = Units.inchesToMeters(20.375);
    public static double camRearRightPitch = Units.degreesToRadians(-15);
    public static double camRearRightYaw = Units.degreesToRadians(-170);



    // Robot to camera transforms or lets the camera know where they are
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(camFrontLeftX, camFrontLeftY, camFrontLeftZ, new Rotation3d(0.0, camFrontLeftPitch, camFrontLeftYaw));
    public static Transform3d robotToCamera1 = new Transform3d(camFrontRightX, camFrontRightY, camFrontRightZ, new Rotation3d(0.0, camFrontRightPitch, camFrontRightYaw));
    public static Transform3d robotToCamera2 = new Transform3d(camRearLeftX, camRearLeftY, camRearLeftZ, new Rotation3d(0.0,  camRearLeftPitch, camRearLeftYaw));
    public static Transform3d robotToCamera3 = new Transform3d(camRearRightX, camRearRightY, camRearRightZ, new Rotation3d(0.0, camRearRightPitch, camRearRightYaw));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
}
