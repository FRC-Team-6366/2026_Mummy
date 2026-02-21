package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO{
    TalonFX leadShooterMotor;
    TalonFX followShooterMotor;
    
    VelocityVoltage velocityVoltageRequest;
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


 public ShooterIOTalonFX(){
    leadShooterMotor = new TalonFX(Constants.ShooterConstants.leadShooterMotorId);   
 
    leadShooterVolts = leadShooterMotor.getMotorVoltage();
    leadShooterPosition = leadShooterMotor.getPosition();
    leadShooterRps = leadShooterMotor.getVelocity();
    leadShooterCurrent = leadShooterMotor.getTorqueCurrent();
    leadShooterSupplyCurrent = leadShooterMotor.getSupplyCurrent();

    TalonFXConfiguration leadcfg = new TalonFXConfiguration();
    leadcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leadcfg.Slot0.kP = 0.5;
    leadcfg.Slot0.kI = 0;
    leadcfg.Slot0.kD = 0;
    this.leadShooterMotor.getConfigurator().apply(leadcfg);
    
    this.velocityVoltageRequest= new VelocityVoltage(0);
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
        followShooterSupplyCurrent);

        // Forcing optimal use of the CAN Bus for this subsystems
        // hardware
    leadShooterMotor.optimizeBusUtilization(0.0, 1.0);
    followShooterMotor.optimizeBusUtilization(0.0, 1.0);


}

    @Override
    public void setShooterPower(double power) {
       double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        leadShooterMotor.setControl(volts);
        followShooterMotor.setControl(volts);
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
        followShooterSupplyCurrent).isOK();
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

    }

    @Override
    public void setShooterVelocity(double rps) {
        this.leadShooterMotor.setControl(velocityVoltageRequest.withVelocity(rps));
        this.followShooterMotor.setControl(this.follower);
    }


}
