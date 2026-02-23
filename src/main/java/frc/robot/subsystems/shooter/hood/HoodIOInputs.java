package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HoodIOInputs {
    boolean connected = false;
    
    // For tracking Hardware statuses
    double hoodVolts = 0;
    double hoodPosition = 0;
    double hoodRps = 0;
    double hoodCurrent = 0;
    double hoodSupplyCurrent = 0;
    
    // For tracking Setpoint statuses
    double hoodPositionSetpoint = 0;
    double hoodPositionError = 0;
    boolean hoodAtSetpoint = false;
}
