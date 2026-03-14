package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterIOInputs {
  boolean connected = false;

  // For tracking Hardware statuses
  double rightLeadShooterVolts = 0;
  double rightLeadShooterPosition = 0;
  double rightLeadShooterRps = 0;
  double rightLeadShooterCurrent = 0;
  double rightLeadShooterSupplyCurrent = 0;

  double rightFollowShooterVolts = 0;
  double rightFollowShooterPosition = 0;
  double rightFollowShooterRps = 0;
  double rightFollowShooterCurrent = 0;
  double rightFollowShooterSupplyCurrent = 0;

  // For tracking Setpoint statuses
  double rightShooterVelocitySetpoint = 0;
  double rightShooterVelocityError = 0;
  boolean rightShooterAtVelocitySetpoint = false;

  double leftLeadShooterVolts = 0;
  double leftLeadShooterPosition = 0;
  double leftLeadShooterRps = 0;
  double leftLeadShooterCurrent = 0;
  double leftLeadShooterSupplyCurrent = 0;

  double leftFollowShooterVolts = 0;
  double leftFollowShooterPosition = 0;
  double leftFollowShooterRps = 0;
  double leftFollowShooterCurrent = 0;
  double leftFollowShooterSupplyCurrent = 0;

  // For tracking Setpoint statuses
  double leftShooterVelocitySetpoint = 0;
  double leftShooterVelocityError = 0;
  boolean leftShooterAtVelocitySetpoint = false;


}
