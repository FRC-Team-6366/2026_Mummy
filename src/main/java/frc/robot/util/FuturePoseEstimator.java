package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.driveTrain.Drive;

public class FuturePoseEstimator {
 Pose2d virtualTarget;
 Transform2d movingTargetTransformation;


  public double getDistanceToHub(Drive drive) {
    boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
    // Select correct dummy pose
    Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;

    // Get the current pose relative to the dummy hub pose. Measurements are from hub to pose
    Pose2d hubToPose = drive.getPose().relativeTo(hubPose);
    double hubToPoseX = hubToPose.getX();
    double hubToPoseY = hubToPose.getY();

    // Calculate distance to hub
    double distanceToHub = Math.sqrt((hubToPoseY*hubToPoseY) + (hubToPoseX*hubToPoseX));
    return distanceToHub;
   }
 
    Pose2d getFutureChasisPose(Drive drive, double BallTime){
        //     boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
        // // Select correct dummy pose
        // Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;

        //Trying to get the Chassis Velocity relative to field
        ChassisSpeeds chassisSpeed = drive.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeed, drive.getRotation());

        // Get the current pose relative to the field
        double virtualTargetX = fieldRelativeSpeeds.vxMetersPerSecond *BallTime;
        double virtualTargetY = fieldRelativeSpeeds.vyMetersPerSecond *BallTime;
        movingTargetTransformation = new Transform2d(virtualTargetX, virtualTargetY, new Rotation2d(0));
        Pose2d futureRobotPose = drive.getPose().transformBy(movingTargetTransformation);
        return futureRobotPose;
      //  return virtualTarget = hubPose.transformBy(virtulTargetTransformation);
  }

  
   public Pose2d getMovingHubPose(Drive drive, double BallTime, Pose2d hubPose){
        //     boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
        // // Select correct dummy pose
        // Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;

        //Trying to get the Chassis Velocity relative to field
        ChassisSpeeds chassisSpeed = drive.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeed, drive.getRotation());

        // Get the current pose relative to the field
        double movingTargetX = -fieldRelativeSpeeds.vxMetersPerSecond *BallTime;
        double movingTargetY = -fieldRelativeSpeeds.vyMetersPerSecond *BallTime;


        movingTargetTransformation = new Transform2d(movingTargetX, movingTargetY, new Rotation2d(0));
        Pose2d movingHubPose = hubPose.transformBy(movingTargetTransformation);

        
        return movingHubPose;
      //  return virtualTarget = hubPose.transformBy(virtulTargetTransformation);
  }

  

  public FuturePoseEstimator(){}



}
