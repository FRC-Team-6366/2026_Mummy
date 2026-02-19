package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO{
    TalonFX leadShooterMotor;
    TalonFX followShooterMotor;
    
    // CANcoder hoodEncoder; 
    //Put anything doing with an encoder in comments because we are not having it on the robot it seems
    TalonFX hoodMotor;

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

    StatusSignal<Voltage> hoodVolts;
    StatusSignal<Angle> hoodPosition;
    StatusSignal<AngularVelocity> hoodRps;
    StatusSignal<Current> hoodCurrent;
    StatusSignal<Current> hoodSupplyCurrent; 
    // StatusSignal<Angle> hoodCANPositionRotations;
    // StatusSignal<AngularVelocity> hoodCANVelocityRps;


ShooterIOTalonFX(){
    leadShooterMotor = new TalonFX(Constants.ShooterConstants.leadShooterMotorId);   
 
    leadShooterVolts = leadShooterMotor.getMotorVoltage();
    leadShooterPosition = leadShooterMotor.getPosition();
    leadShooterRps = leadShooterMotor.getVelocity();
    leadShooterCurrent = leadShooterMotor.getTorqueCurrent();
    leadShooterSupplyCurrent = leadShooterMotor.getSupplyCurrent();

    TalonFXConfiguration leadcfg = new TalonFXConfiguration();
    leadcfg.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    this.leadShooterMotor.getConfigurator().apply(leadcfg);


    followShooterMotor = new TalonFX(Constants.ShooterConstants.followerShooterMotorId);

    followShooterVolts = followShooterMotor.getMotorVoltage();
    followShooterPosition = followShooterMotor.getPosition();
    followShooterRps = followShooterMotor.getVelocity();
    followShooterCurrent = followShooterMotor.getTorqueCurrent();
    followShooterSupplyCurrent = followShooterMotor.getSupplyCurrent();

    TalonFXConfiguration followcfg = new TalonFXConfiguration();
    followcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    this.followShooterMotor.getConfigurator().apply(followcfg);


    hoodMotor = new TalonFX(Constants.ShooterConstants.hoodMotorId);
    // hoodEncoder = new CANcoder(Constants.ShooterConstants.hoodMotorCANCoderId);

    hoodVolts = hoodMotor.getMotorVoltage();
    hoodPosition = hoodMotor.getPosition();
    hoodRps = hoodMotor.getVelocity();
    hoodCurrent = hoodMotor.getTorqueCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    // hoodCANPositionRotations = hoodEncoder.getAbsolutePosition();
    // hoodCANVelocityRps = hoodEncoder.getVelocity();

    TalonFXConfiguration hoodcfg = new TalonFXConfiguration();
    hoodcfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    this.followShooterMotor.getConfigurator().apply(hoodcfg);

    // hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble());

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
        hoodVolts,
        hoodPosition,
        hoodRps,
        hoodCurrent,
        hoodSupplyCurrent
        // hoodCANPositionRotations,
        // hoodCANVelocityRps
        );

    leadShooterMotor.optimizeBusUtilization(0.0, 1.0);
    followShooterMotor.optimizeBusUtilization(0.0, 1.0);

    hoodMotor.optimizeBusUtilization(0.0, 1.0);
    // hoodEncoder.optimizeBusUtilization(0.0, 1.0);
}

    @Override
    public void setShooterPower(double power) {
       double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        leadShooterMotor.setControl(volts);
        followShooterMotor.setControl(volts);
    }



    // @Override
    // public void setHoodAngle(double angle){
    //  hoodMotor.setControl(hoodPosition.withPosition(getValueAsDouble()));
    // }


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
        hoodVolts,
        hoodPosition,
        hoodRps,
        hoodCurrent,
        hoodSupplyCurrent
        // hoodCANPositionRotations,
        // hoodCANVelocityRps
        ).isOK();
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

        inputs.hoodVolts = this.hoodVolts.getValueAsDouble();
        inputs.hoodPosition = this.hoodPosition.getValueAsDouble();
        inputs.hoodRps = this.hoodRps.getValueAsDouble();
        inputs.hoodCurrent = this.hoodCurrent.getValueAsDouble();
        inputs.hoodSupplyCurrent = this.hoodSupplyCurrent.getValueAsDouble();
        // inputs.hoodCANPositionRotations = this.hoodCANPositionRotations.getValueAsDouble();
        // inputs.hoodCANVelocityRps = this.hoodCANVelocityRps.getValueAsDouble();
    }


}
