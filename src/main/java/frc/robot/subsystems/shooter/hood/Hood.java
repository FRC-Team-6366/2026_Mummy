package frc.robot.subsystems.shooter.hood;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.driveTrain.Drive;

public class Hood extends SubsystemBase {
    HoodIO hoodIO;
    double angle = 0;


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
     * @return Command to set hood to starting position
     */
    public Command retractHood() {
        this.angle = 0;
        return this.hoodToAngle(this.angle);
    }
    
    /**
     * Sets the hood for shooting at specified angle
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @param angle Angle in degrees
     * @return Command to set hood at angle for shooting
     */
    public Command hoodToAngle(double angle) {
        return this.run(
            () -> {
                this.angle = angle;
                this.hoodIO.hoodToAngle(this.angle);
            });
    }

    public Command hoodIncrements(){
        return this.runOnce(
         () ->
         { 
            this.angle += 1;
            this.angle = MathUtil.clamp(this.angle, 0.0, 45.0);
            this.hoodIO.hoodToAngle(this.angle);
         });
    }

    public Command hoodDecrements(){
        return this.runOnce(
         () ->
         { 
            this.angle -= 1;
         this.angle = MathUtil.clamp(this.angle, 0.0, 45.0);
            this.hoodIO.hoodToAngle(this.angle);
         });
    }

    /**
     * Sets the hood for shooting at hanging station
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for close shooting
     */
    public Command hoodToAnglePosition1(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition1Angle);
    }

    /**
     * Sets the hood for shooting at trench wall
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for medium shooting
     */
    public Command hoodToAnglePosition2(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition2Angle);
    }

    /**
     * Sets the hood for shooting at human player station
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for far shooting
     */
    public Command hoodToAnglePosition3(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition3Angle);
    }

    /**
     * Automatically sets the hood for shooting from any distance
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for auto calculated shooting
     */
    public Command setHoodAutoAngle(Drive drive){
        // Construct command
        return Commands.run(
            () -> {
                // Check for alliance side
                // boolean isFlipped =
                //     DriverStation.getAlliance().isPresent()
                //         && DriverStation.getAlliance().get() == Alliance.Red;

                boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                
                // Select correct dummy pose
                Pose2d hubPose = isFlipped ? Constants.PoseConstants.hubPoseRed : Constants.PoseConstants.hubPoseBlue;
                // Get the current pose relative to the dummy hub pose. Measurements are from hub to pose
                Pose2d hubToPose = drive.getPose().relativeTo(hubPose);
                double hubToPoseX = hubToPose.getX();
                double hubToPoseY = hubToPose.getY();
                // Find the hypotenuse of the triangle
                double distanceToHub = Math.sqrt((hubToPoseX * hubToPoseX) + (hubToPoseY * hubToPoseY));

                this.angle = hoodAngleMap.get(distanceToHub);
                this.hoodIO.hoodToAngle(angle);
                // hubPose.getTranslation().getDistance(drive.getPose().getTranslation());
            }
        );
    }

    /**
     * Returns whether the hood is at its set point distance, given a percent of tolerence
     * specified in the HoodIO hardware class
     * @return BooleanSupplier: True hood is at its setpoint, false otherwise
     */
    public BooleanSupplier hoodAtPositionSetpoint() {
        return () -> this.hoodIO.hoodAtPositionSetpoint();
    }

    @Override
    public void periodic() {
        this.hoodIO.updateInputs(inputs);
        Logger.processInputs("HoodSubsystem", inputs);
        Logger.recordOutput("HoodSubsystem/Alliance", DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
