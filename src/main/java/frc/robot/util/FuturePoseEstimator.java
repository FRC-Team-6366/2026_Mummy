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
 Drive drive;
 Pose2d virtualTarget;
 Transform2d virtulTargetTransformation;


   Transform2d robotChassisPose(){
        //     boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
        // // Select correct dummy pose
        // Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;

        //Trying to get the Chassis Velocity relative to field
        ChassisSpeeds chassisSpeed = drive.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeed, drive.getRotation());

        // Get the current pose relative to the field
        double virtualTargetX = fieldRelativeSpeeds.vxMetersPerSecond;
        double virtualTargetY = fieldRelativeSpeeds.vyMetersPerSecond;
        return virtulTargetTransformation = new Transform2d(virtualTargetX, virtualTargetY, new Rotation2d(0));
      //  return virtualTarget = hubPose.transformBy(virtulTargetTransformation);
  }

  

  public FuturePoseEstimator(){}



}
