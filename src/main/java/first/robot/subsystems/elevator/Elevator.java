package first.robot.subsystems.elevator;

import first.robot.util.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color8Bit;

public class Elevator implements Mechanism {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Elevator", "MotorDisconnected", "Disconnected arm motor.", Level.HIGH);

  private final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(1.2, 1.2);
  private final LoggedMechanismLigament2d measuredLigament =
      mechanism2d
          .getRoot("Elevator", 0.6, 0.6)
          .append(
              new LoggedMechanismLigament2d(
                  "Measured",
                  ElevatorConstants.LOW_HEIGHT_ELEVATOR_M,
                  0.0,
                  6,
                  new Color8Bit(255, 165, 0)));

  // Editable from the dashboard under /Tuning/Arm/ in tuning mode.
  private final TunableNumber kP = new TunableNumber("Elevator/kP", ElevatorConstants.GAINS.kP());
  private final TunableNumber kD = new TunableNumber("Elevator/kD", ElevatorConstants.GAINS.kD());
  private final TunableNumber kG = new TunableNumber("Elevator/kG", ElevatorConstants.GAINS.kG());
  private final TunableNumber cruiseVelocity =
      new TunableNumber("Elevator/CruiseVelocity", ElevatorConstants.GAINS.cruiseVelocity());
  private final TunableNumber acceleration =
      new TunableNumber("Elevator/Acceleration", ElevatorConstants.GAINS.acceleration());
  // What the motor controller currently has, so gains are only re-sent when one changes.
  private ElevatorIO.Gains appliedGains = ElevatorConstants.GAINS;

  private double goalPos = Double.NaN;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    var gains =
        new ElevatorIO.Gains(
            kP.get(), kD.get(), kG.get(), cruiseVelocity.get(), acceleration.get());
    if (!gains.equals(appliedGains)) {
      io.setGains(gains);
      appliedGains = gains;
    }

    if (RobotState.isDisabled()) {
      goalPos = inputs.positionM;
    }

    // if (io.isZeroed()) {
    //   io.limitHit();
    // }

    measuredLigament.setLength(inputs.positionM);
    Logger.recordOutput("Elevator/Mechanism2d", mechanism2d);

    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  public Command goTo(double positionM) {
    double clampedRad =
        Math.clamp(
            positionM,
            ElevatorConstants.LOW_HEIGHT_ELEVATOR_M,
            ElevatorConstants.MAX_HEIGHT_ELEVATOR_M);
    return run(coroutine -> {
          goalPos = clampedRad;
          while (!isAtGoal()) {
            io.setPosition(goalPos);
            coroutine.yield();
          }
        })
        .named("Elevator.GoTo[" + positionM + " M]");
  }

  public Command setVoltage(double voltage) {
    return run(coroutine -> {
          io.setVoltage(voltage);
        })
        .named("Set Voltage");
  }

  /** Keeps commanding the current goal. Never finishes; meant as the default command. */
  public Command hold() {
    return runRepeatedly(
            () -> {
              if (Double.isNaN(goalPos)) {
                return;
              }
              if (goalPos < ElevatorConstants.LOW_HEIGHT_ELEVATOR_M
                  || goalPos > ElevatorConstants.MAX_HEIGHT_ELEVATOR_M) {
                io.setVoltage(0.0);
              } else {
                io.setPosition(goalPos);
              }
            })
        .named("Elevator.Hold");
  }

  @AutoLogOutput(key = "Elevator/GoalM")
  public double getGoalPos() {
    return goalPos;
  }

  @AutoLogOutput(key = "Elevator/GetPosition")
  public double getPositionM() {
    return inputs.positionM;
  }

  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean isAtGoal() {
    return Math.abs(inputs.positionM - goalPos) < ElevatorConstants.ELEVATOR_TOLERANCE;
  }
}
