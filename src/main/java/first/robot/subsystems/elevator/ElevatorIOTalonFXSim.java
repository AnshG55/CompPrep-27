package first.robot.subsystems.elevator;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  public ElevatorIOTalonFXSim(ElevatorSimulation elevatorSim) {
    super();
    elevatorSim.attach(motor.getSimState());
  }
}
