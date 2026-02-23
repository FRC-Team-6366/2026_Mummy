package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
    TalonFX leadShooterMotor;
    TalonFX followShooterMotor;

    // Used to control motor output by specifying rotaion speed
    VelocityVoltage velocityVoltageRequest;
    // Used to make followShooterMotor mimic the leadShooterMotor
    Follower follower;

    StatusSignal<Voltage> leadShooterVolts;
    StatusSignal<Angle> leadShooterPosition;
    StatusSignal<AngularVelocity> leadShooterRps;
    StatusSignal<Current> leadShooterCurrent;
    StatusSignal<Current> leadShooterSupplyCurrent;

    StatusSignal<Voltage> followShooterVolts;
    StatusSignal<Angle> followShooterPosition;
    StatusSignal<AngularVelocity> followShooterRps;
    StatusSignal<Current> followShooterCurrent;
    StatusSignal<Current> followShooterSupplyCurrent;

    StatusSignal<Double> shooterVelocityError;

    // Speficy min and max velocity for adjusting
    // other elements of the shooter subsystem such as tolerance amount
    public static final double shooterMinVelocityRPS = 0;
    public static final double shooterMaxVelocityRPS = 6000;

    
    // Setpoint tracking variables
    /**
     * How far the shooter's velocity can be off and still be concidered at setpoint.
     * Valid values from 0.0 (no tolerance!) to 100.0 (no accuracy!)
     */
    double setPointTolerancePercent = 1;
    double setPointTolerance;
    double velocitySetPointLow;
    double velocitySetPointHigh;

    public ShooterIOTalonFX() {
        // Instantiating Lead Shooter motor and its variables for monitoring
        leadShooterMotor = new TalonFX(Constants.ShooterConstants.leadShooterMotorId);
        leadShooterVolts = leadShooterMotor.getMotorVoltage();
        leadShooterPosition = leadShooterMotor.getPosition();
        leadShooterRps = leadShooterMotor.getVelocity();
        leadShooterCurrent = leadShooterMotor.getTorqueCurrent();
        leadShooterSupplyCurrent = leadShooterMotor.getSupplyCurrent();
        shooterVelocityError = leadShooterMotor.getClosedLoopError();

        // Instantiating configuration for Lead Shooter motor
        TalonFXConfiguration leadcfg = new TalonFXConfiguration();
        leadcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        leadcfg.Slot0.kP = 0.5;
        leadcfg.Slot0.kI = 0;
        leadcfg.Slot0.kD = 0;
        this.leadShooterMotor.getConfigurator().apply(leadcfg);

        // Instantiating velocity voltage object for setting the output velocity
        this.velocityVoltageRequest = new VelocityVoltage(0);
        this.leadShooterMotor.setControl(velocityVoltageRequest.withSlot(0));

        // Setting the StatusSignal variables to be mapped to actual
        // aspect of the ShooterIO's hardware
        followShooterMotor = new TalonFX(Constants.ShooterConstants.followerShooterMotorId);
        followShooterVolts = followShooterMotor.getMotorVoltage();
        followShooterPosition = followShooterMotor.getPosition();
        followShooterRps = followShooterMotor.getVelocity();
        followShooterCurrent = followShooterMotor.getTorqueCurrent();
        followShooterSupplyCurrent = followShooterMotor.getSupplyCurrent();
        follower = new Follower(Constants.ShooterConstants.leadShooterMotorId, MotorAlignmentValue.Opposed);
        this.followShooterMotor.setControl(follower);

        // Set update period for device metrics to be 50 Hz (20 milliseconds)
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leadShooterVolts,
                leadShooterPosition,
                leadShooterRps,
                leadShooterCurrent,
                leadShooterSupplyCurrent,
                followShooterVolts,
                followShooterPosition,
                followShooterRps,
                followShooterCurrent,
                followShooterSupplyCurrent,
                shooterVelocityError
        );

        // Forcing optimal use of the CAN Bus for this subsystems
        // hardware
        leadShooterMotor.optimizeBusUtilization(0.0, 1.0);
        followShooterMotor.optimizeBusUtilization(0.0, 1.0);
        
        this.setPointTolerance = shooterMaxVelocityRPS * this.setPointTolerancePercent;
    }

    @Override
    public void setShooterVelocityRPS(double rps) {
        double rpsToUse = MathUtil.clamp(rps, shooterMinVelocityRPS, shooterMaxVelocityRPS);
        this.leadShooterMotor.setControl(velocityVoltageRequest.withVelocity(rpsToUse));
        this.followShooterMotor.setControl(this.follower);
    }

   @Override
    public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
        // double wheelDiameterInInches = 4;
        // double wheelDiameterInFeet = wheelDiameterInInches / 12;
        // double wheelCircumferenceInFeet = wheelDiameterInFeet * Math.PI;
        // double rotationsPerSecond = feetPerSecond / wheelCircumferenceInFeet;
        double rotationsPerSecond = feetPerSecond / ((4 / 12) * Math.PI);
        double rpsToUse = MathUtil.clamp(rotationsPerSecond, shooterMinVelocityRPS, shooterMaxVelocityRPS);

        this.leadShooterMotor.setControl(velocityVoltageRequest.withVelocity(rpsToUse));
        this.followShooterMotor.setControl(this.follower);
    }

    @Override
    public double getShooterVelocityError() {
        return this.leadShooterMotor.getClosedLoopError().getValueAsDouble();
    }

    @Override
    public boolean shooterAtVelocitySetPoint() {
        // Get absolute value of the error and see if it is less
        // than the setpoint tolerance
        return Math.abs(this.getShooterVelocityError()) < this.setPointTolerance;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                leadShooterVolts,
                leadShooterPosition,
                leadShooterRps,
                leadShooterCurrent,
                leadShooterSupplyCurrent,
                followShooterVolts,
                followShooterPosition,
                followShooterRps,
                followShooterCurrent,
                followShooterSupplyCurrent,
                shooterVelocityError
        ).isOK();
        
        // Update Hardware fields
        inputs.leadShooterVolts = this.leadShooterVolts.getValueAsDouble();
        inputs.leadShooterPosition = this.leadShooterPosition.getValueAsDouble();
        inputs.leadShooterRps = this.leadShooterRps.getValueAsDouble();
        inputs.leadShooterCurrent = this.leadShooterCurrent.getValueAsDouble();
        inputs.leadShooterSupplyCurrent = this.leadShooterSupplyCurrent.getValueAsDouble();

        inputs.followShooterVolts = this.followShooterVolts.getValueAsDouble();
        inputs.followShooterPosition = this.followShooterPosition.getValueAsDouble();
        inputs.followShooterRps = this.followShooterRps.getValueAsDouble();
        inputs.followShooterCurrent = this.followShooterCurrent.getValueAsDouble();
        inputs.followShooterSupplyCurrent = this.followShooterSupplyCurrent.getValueAsDouble();

        // Update Setpoint related fields
        inputs.shooterVelocitySetpoint = this.velocityVoltageRequest.Velocity;
        inputs.shooterVelocityError = this.shooterVelocityError.getValueAsDouble();
        inputs.shooterAtVelocitySetpoint = this.shooterAtVelocitySetPoint();
    }

}
