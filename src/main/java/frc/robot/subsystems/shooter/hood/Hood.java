package frc.robot.subsystems.shooter.hood;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.driveTrain.Drive;

public class Hood extends SubsystemBase {
  HoodIO hoodIO;
  double angle = 0;
  double angleOfHood;

  HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  public Hood(HoodIO io) {
    this.hoodIO = io;

    // Interpolation map to calculate hood angle for any given distance
    // Starting with values from three set points
    hoodAngleMap.put(1.01, 0.0);
    hoodAngleMap.put(1.596, 22.0);
    hoodAngleMap.put(3.369, 25.17);
    hoodAngleMap.put(4.004, 36.3);
  }

  /**
   * Retracts the hood to its starting position
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @return Command to set hood to starting position
   */
  public Command retractHood() {
    return this.hoodsToAngle(15).withName("retractHood()");
  }

  /**
   * Sets the hood for shooting at specified angle
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @param angle
   *          Angle in degrees
   * @return Command to set hood at angle for shooting
   */
  public Command hoodsToAngle(double angle) {
    return this.run(
        () -> {
          this.angle = angle;
          this.hoodIO.hoodsToAngle(this.angle);
        }).withName("hoodsToAngle()");
  }



  public double gotHoodPosition(){
    angleOfHood =  hoodIO.getHoodPosition();
    return angleOfHood;
  }
  /**
   * Sets both the left and rightt hoods for shooting at tower station
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @return Command to set hood for close shooting
   */
  public Command hoodToAnglePosition1() {
    return this.hoodsToAngle(Constants.ShooterConstants.hoodPosition1Angle).withName("hoodToAnglePosition1()");
  }

  /**
   * Sets both the left and rightt hoods for shooting at trench wall
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @return Command to set hood for medium shooting
   */
  public Command hoodToAnglePosition2() {
    return this.hoodsToAngle(Constants.ShooterConstants.hoodPosition2Angle).withName("hoodToAnglePosition2()");
  }

  /**
   * Sets both the left and rightt hoods for shooting at human player station
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @return Command to set hood for far shooting
   */
  public Command hoodToAnglePosition3() {
    return this.hoodsToAngle(Constants.ShooterConstants.hoodPosition3Angle).withName("hoodToAnglePosition3()");
  }

  /**
   * Automatically sets the hoods for shooting from any distance
   * <p>
   * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
   * 
   * @return Command to set hood for auto calculated shooting
   */
  public Command setHoodAutoAngle(Drive drive) {
    // Construct command
    return this.run(
        () -> {
          // Check for alliance side
          boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

          // Select correct dummy pose
          Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;
          // Get the current pose relative to the dummy hub pose. Measurements are from
          // hub to pose
          Pose2d hubToPose = drive.getPose().relativeTo(hubPose);
          double hubToPoseX = hubToPose.getX();
          double hubToPoseY = hubToPose.getY();
          // Find the hypotenuse of the triangle
          double distanceToHub = Math.sqrt((hubToPoseX * hubToPoseX) + (hubToPoseY * hubToPoseY));

          this.angle = hoodAngleMap.get(distanceToHub);
          this.hoodIO.hoodsToAngle(angle);
          // hubPose.getTranslation().getDistance(drive.getPose().getTranslation());
        }).withName("setHoodAutoAngle()");
  }

  /**
   * Returns whether the hood is at its set point distance, given a percent of
   * tolerence
   * specified in the HoodIO hardware class
   * 
   * @return BooleanSupplier: True hood is at its setpoint, false otherwise
   */
  public BooleanSupplier hoodAtPositionSetpoint() {
    return () -> this.hoodIO.hoodsAtPositionSetpoint();
  }

  @Override
  public void periodic() {
    this.hoodIO.updateInputs(inputs);
    Logger.processInputs("HoodSubsystem", inputs);
    Logger.recordOutput("HoodSubsystem/Alliance", DriverStation.getAlliance().orElse(Alliance.Blue));
    Logger.recordOutput("HoodSubsystem/DefaultCommand",
        this.getDefaultCommand() != null ? this.getDefaultCommand().getName() : "N/A");
    Logger.recordOutput("HoodSubsystem/CurrentCommand", 
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "N/A");
    Logger.recordOutput("HoodTrueAngle", this.gotHoodPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
