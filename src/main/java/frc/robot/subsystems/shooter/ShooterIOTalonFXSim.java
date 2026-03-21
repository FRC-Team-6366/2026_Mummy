package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOTalonFXSim extends ShooterIOTalonFX {
// ---- Simulation objects (only used in sim) ----
  FlywheelSim rightFlywheelSim;
  FlywheelSim leftFlywheelSim;
  TalonFXSimState rightLeadShooterMotorSimState;
  TalonFXSimState rightFollowerShooterMotorSimState;
  TalonFXSimState leftLeadShooterMotorSimState;
  TalonFXSimState leftFollowerShooterMotorSimState;

  public ShooterIOTalonFXSim () {
    // Call constructor for parent class
    super();
    
    // Instantiate the flywheel object that simulates the shooter wheel
    this.rightFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2), 
        0.001, 
        1.0
        ), 
      DCMotor.getKrakenX60(2), 
      0.004
    );

    this.leftFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2), 
        0.001, 
        1.0
        ), 
      DCMotor.getKrakenX60(2), 
      0.004
    );
    
    // Instantiate the motor simulated states for reporting to simulators
    this.rightLeadShooterMotorSimState = this.rightLeadShooterMotor.getSimState();
    this.rightFollowerShooterMotorSimState = this.rightFollowerShooterMotor.getSimState();
    this.leftLeadShooterMotorSimState = this.leftLeadShooterMotor.getSimState();
    this.leftFollowerShooterMotorSimState = this.leftFollowerShooterMotor.getSimState();
  }

  public void updateSimulation() {
    // 1. Feed current battery voltage to sim states
    this.rightLeadShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.rightFollowerShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.leftLeadShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.leftFollowerShooterMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2. Read the voltage the leader motor is actually applying
    double rightLeadShooterMotorVoltage = this.rightLeadShooterMotorSimState.getMotorVoltage();
    double leftLeadShooterMotorVoltage = this.leftLeadShooterMotorSimState.getMotorVoltage();

    // 3. Update the physics model
    this.rightFlywheelSim.setInput(rightLeadShooterMotorVoltage);
    this.rightFlywheelSim.update(0.020);
    this.leftFlywheelSim.setInput(leftLeadShooterMotorVoltage);
    this.leftFlywheelSim.update(0.020);


    // 4. Write simulated velocity/position back into the TalonFX sim states
    double rightFlywheelRPS = rightFlywheelSim.getAngularVelocityRPM() / 60;
    double leftFlywheelRPS = leftFlywheelSim.getAngularVelocityRPM() / 60;

    this.rightLeadShooterMotorSimState.setRotorVelocity(rightFlywheelRPS);
    this.rightLeadShooterMotorSimState.setRotorVelocity(-rightFlywheelRPS);
    this.leftLeadShooterMotorSimState.setRotorVelocity(leftFlywheelRPS);
    this.leftFollowerShooterMotorSimState.setRotorVelocity(-leftFlywheelRPS);

    this.rightLeadShooterMotorSimState.addRotorPosition(rightFlywheelRPS * 0.020);
    this.rightFollowerShooterMotorSimState.addRotorPosition(-rightFlywheelRPS * 0.020);
    this.leftLeadShooterMotorSimState.addRotorPosition(leftFlywheelRPS * 0.020);
    this.leftFollowerShooterMotorSimState.addRotorPosition(-leftFlywheelRPS * 0.020);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    this.updateSimulation();
    super.updateInputs(inputs);
  }
}
