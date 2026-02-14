package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Voltage; //  Voltage
import edu.wpi.first.units.measure.AngularVelocity; // Rotations Per Second
import edu.wpi.first.units.measure.Angle; // position
import edu.wpi.first.units.measure.Current;// Supply current


public class IndexerIOTalonFX implements IndexerIO{
    private final TalonFX indexMotor;
    private final TalonFX indexWallMotor;
    
    StatusSignal<Voltage> indexVolts;
    StatusSignal<Angle> indexPosition;
    StatusSignal<AngularVelocity> indexRps;
    StatusSignal<Current> indexCurrent;
    StatusSignal<Current> indexSupplyCurrent; 

    StatusSignal<Voltage> indexWallVolts;
    StatusSignal<Angle> indexWallPosition;
    StatusSignal<AngularVelocity> indexWallRps;
    StatusSignal<Current> indexWallCurrent;
    StatusSignal<Current> indexWallSupplyCurrent; 


    public IndexerIOTalonFX(){
        indexMotor= new TalonFX(38); //16
        TalonFXConfiguration indMotorconfiguration = new TalonFXConfiguration();
        indMotorconfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        indexMotor.getConfigurator().apply(indMotorconfiguration);

        indexWallMotor= new TalonFX(39);//STAND IN MOTOR, ask for wall motor device Id later
        indMotorconfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        indexWallMotor.getConfigurator().apply(indMotorconfiguration);

        indexVolts = indexMotor.getMotorVoltage();
        indexPosition = indexMotor.getPosition();
        indexRps = indexMotor.getVelocity();
        indexCurrent = indexMotor.getTorqueCurrent();
        indexSupplyCurrent = indexMotor.getSupplyCurrent();

        indexWallVolts = indexWallMotor.getMotorVoltage();
        indexWallPosition = indexWallMotor.getPosition();
        indexWallRps = indexWallMotor.getVelocity();
        indexWallCurrent = indexWallMotor.getTorqueCurrent();
        indexWallSupplyCurrent = indexWallMotor.getSupplyCurrent();

        
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        indexVolts,
        indexPosition,
        indexRps,
        indexCurrent,
        indexSupplyCurrent,
        indexWallVolts,
        indexPosition,
        indexWallRps,
        indexWallCurrent,
        indexWallSupplyCurrent);

    indexMotor.optimizeBusUtilization(0.0, 1.0);
    indexWallMotor.optimizeBusUtilization(0.0, 1.0);

    }

    @Override
    public void setIndexerPower(double power) {
       double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        indexMotor.setControl(volts);
    }

    @Override
    public void setIndexWallPower(double power) {
        double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        indexWallMotor.setControl(volts);
    }

    @Override 
    public void updateInputs(IndexerIOInputs inputs){
     inputs.connected = BaseStatusSignal.refreshAll(
        indexVolts,
        indexPosition,
        indexRps,
        indexCurrent,
        indexSupplyCurrent,
        indexWallVolts,
        indexWallPosition,
        indexWallRps,
        indexWallCurrent,
        indexWallSupplyCurrent).isOK();
        inputs.indexVolts = this.indexVolts.getValueAsDouble();
        inputs.indexPosition = this.indexPosition.getValueAsDouble();
        inputs.indexRps = this.indexRps.getValueAsDouble();
        inputs.indexCurrent = this.indexCurrent.getValueAsDouble();
        inputs.indexSupplyCurrent = this.indexSupplyCurrent.getValueAsDouble();

        inputs.indexWallVolts = this.indexWallVolts.getValueAsDouble();
        inputs.indexWallPosition = this.indexWallPosition.getValueAsDouble();
        inputs.indexWallRps = this.indexWallRps.getValueAsDouble();
        inputs.indexWallCurrent = this.indexWallCurrent.getValueAsDouble();
        inputs.indexWallSupplyCurrent = this.indexWallSupplyCurrent.getValueAsDouble();
    }
    
    

}
