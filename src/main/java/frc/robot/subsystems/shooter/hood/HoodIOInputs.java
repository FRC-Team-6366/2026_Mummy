package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HoodIOInputs {
  boolean connected = false;

  // For tracking Hardware statuses
  double hoodVoltsRight = 0;
  double hoodPositionRight = 0;
  double hoodRPSRight = 0;
  double hoodCurrentRight = 0;
  double hoodSupplyCurrentRight = 0;

  double hoodVoltsLeft = 0;
  double hoodPositionLeft = 0;
  double hoodRPSLeft = 0;
  double hoodCurrentLeft = 0;
  double hoodSupplyCurrentLeft = 0;

  // For tracking Setpoint statuses
  double hoodPositionSetpointRight = 0;
  double hoodPositionErrorRight = 0;
  boolean hoodAtSetpointRight = false;

  double hoodPositionSetpointLeft = 0;
  double hoodPositionErrorLeft = 0;
  boolean hoodAtSetpointLeft = false;
}
