package first.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Protected so the sim subclass can hand these devices to its sim model.
  protected final TalonFX motor =
      new TalonFX(ElevatorConstants.MOTOR_ID, ElevatorConstants.CAN_BUS);

  protected final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // A single dropped frame shouldn't raise a disconnect alert.
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public ElevatorIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = ElevatorConstants.MOTOR_DIRECTION;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0 = slot0For(ElevatorConstants.GAINS);
    config.MotionMagic = motionMagicFor(ElevatorConstants.GAINS);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_HEIGHT_ELEVATOR_M;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.LOW_HEIGHT_ELEVATOR_M;

    // false skips the refresh Phoenix normally does on lookup. Right after startup the first status
    // frame may not have arrived, and that refresh would log a CAN error. updateInputs refreshes
    // every loop anyway.
    position = motor.getPosition(false);
    velocity = motor.getVelocity(false);
    appliedVolts = motor.getMotorVoltage(false);
    current = motor.getStatorCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    // Turns off every status frame not given an explicit rate above.
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.positionM = position.getValueAsDouble() * ElevatorConstants.ROT_TO_METERS;
    inputs.velocityMPerSec = velocity.getValueAsDouble() * ElevatorConstants.ROT_TO_METERS;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setGains(Gains gains) {
    motor.getConfigurator().apply(slot0For(gains));
    motor.getConfigurator().apply(motionMagicFor(gains));
  }

  private static Slot0Configs slot0For(Gains gains) {
    return new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(gains.kG())
        .withKS(ElevatorConstants.KS)
        .withKV(ElevatorConstants.KV)
        .withKA(ElevatorConstants.KA)
        .withKP(gains.kP())
        .withKD(gains.kD());
  }

  private static MotionMagicConfigs motionMagicFor(Gains gains) {
    return new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(gains.cruiseVelocity())
        .withMotionMagicAcceleration(gains.acceleration());
  }

  @Override
  public void setPosition(double positionM) {
    motor.setControl(positionRequest.withPosition(positionM / ElevatorConstants.GEAR_RATIO));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void limitHit() {
    motor.setPosition(ElevatorConstants.SWITCH_HEIGHT / ElevatorConstants.GEAR_RATIO);
  }

  @Override
  public boolean isZeroed() {
    if (limitSwitch.get()) {
      return true;
    } else {
      return false;
    }
  }
}
