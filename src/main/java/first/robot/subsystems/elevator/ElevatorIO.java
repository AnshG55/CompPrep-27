package first.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    public double positionM = 0.0;

    public double velocityMPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  record Gains(double kP, double kD, double kG, double cruiseVelocity, double acceleration) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setGains(Gains gains) {}

  default void setPosition(double angleRad) {}

  default void setVoltage(double volts) {}

  default void limitHit() {}

  default boolean isZeroed() {
    return false;
  }
}
