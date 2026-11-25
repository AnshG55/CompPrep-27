package first.robot.subsystems.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import first.robot.sim.PhoenixSimUtil;
import first.robot.sim.SimulatedMechanism;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;

public class ElevatorSimulation implements SimulatedMechanism {
  private static final double FUDGE_GRAVITY_SCALE = 1.0;

  private final ElevatorSim elevator =
      new ElevatorSim(
          Models.elevatorFromSysId(ElevatorConstants.KV, ElevatorConstants.KA),
          ElevatorConstants.MOTOR.toDCMotor(1),
          ElevatorConstants.LOW_HEIGHT_ELEVATOR_M / ElevatorConstants.ROT_TO_METERS,
          ElevatorConstants.MAX_HEIGHT_ELEVATOR_M / ElevatorConstants.ROT_TO_METERS,
          false,
          ElevatorConstants.LOW_HEIGHT_ELEVATOR_M / ElevatorConstants.ROT_TO_METERS);

  private TalonFXSimState motorSim = null;

  void attach(TalonFXSimState motorSim) {
    this.motorSim = motorSim;
    motorSim.Orientation = PhoenixSimUtil.orientation(ElevatorConstants.MOTOR_DIRECTION);
    motorSim.setMotorType(MotorType.KrakenX60);
    writeSensors();
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    motorSim.setSupplyVoltage(batteryVolts);
    double gravityVolts = FUDGE_GRAVITY_SCALE * ElevatorConstants.KG;
    double netVolts = motorSim.getMotorVoltage() - gravityVolts;
    double before = elevator.getVelocity();
    elevator.setInputVoltage(PhoenixSimUtil.applyFriction(netVolts, before, ElevatorConstants.KS));
    elevator.update(dtSeconds);
    if (PhoenixSimUtil.stoppedByFriction(
        before, elevator.getVelocity(), netVolts, ElevatorConstants.KS)) {
      elevator.setState(elevator.getPosition(), 0.0);
    }
    writeSensors();
    return motorSim.getSupplyCurrent();
  }

  private void writeSensors() {
    double elevatorPos = elevator.getPosition() / ElevatorConstants.ROT_TO_METERS;
    double elevatorMPerSec = elevator.getVelocity() / ElevatorConstants.ROT_TO_METERS;
    motorSim.setRawRotorPosition(elevatorPos);
    motorSim.setRotorVelocity(elevatorMPerSec);
  }
}
