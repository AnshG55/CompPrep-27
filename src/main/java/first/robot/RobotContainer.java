package first.robot;

import com.ctre.phoenix6.SignalLogger;
import first.robot.autos.Autos;
import first.robot.sim.Battery;
import first.robot.sim.SimWorld;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCharacterization;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.DriveSystemsCheck;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.GyroIOPigeon2Sim;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.drive.ModuleIOTalonFXSim;
import first.robot.subsystems.drive.SwerveDriveSim;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorIOTalonFX;
import first.robot.subsystems.elevator.ElevatorIOTalonFXSim;
import first.robot.subsystems.elevator.ElevatorSimulation;
import first.robot.subsystems.vision.LimelightSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command3.Command;

/**
 * Builds the robot and connects it to the controller and dashboard.
 *
 * <ul>
 *   <li>The constructor picks the IO for each mechanism, depending on whether this is a real robot,
 *       the simulator, or a replay. Skim it; the three cases build the same things.
 *   <li>{@link Controls} (a separate file) connects the gamepad's buttons to commands.
 *   <li>{@link #configureAutos()} lists the autonomous routines for the dashboard.
 *   <li>{@link #periodic()} updates every mechanism once per loop.
 * </ul>
 */
public class RobotContainer {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;

  // Only used in SIM. The first two are null and the list is empty otherwise.
  private final SimWorld simWorld;
  private final SwerveDriveSim driveSim;

  private final LoggedNetworkChooser<Command> autoChooser =
      new LoggedNetworkChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        simWorld = null;
        driveSim = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.MODULES[0]),
                new ModuleIOTalonFX(DriveConstants.MODULES[1]),
                new ModuleIOTalonFX(DriveConstants.MODULES[2]),
                new ModuleIOTalonFX(DriveConstants.MODULES[3]));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[0],
                    VisionConstants.ROBOT_TO_CAMERA[0],
                    drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[1],
                    VisionConstants.ROBOT_TO_CAMERA[1],
                    drive::getRotation));

        elevator = new Elevator(new ElevatorIOTalonFX());
      }
      case SIM -> {
        SignalLogger.enableAutoLogging(false);
        simWorld = new SimWorld(new Battery());
        driveSim = new SwerveDriveSim();
        simWorld.add("Drive", driveSim);
        drive =
            new Drive(
                new GyroIOPigeon2Sim(driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[0], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[1], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[2], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[3], driveSim));

        // Vision needs no sim IO subclass. Each LimelightSim publishes to NetworkTables exactly
        // like a real Limelight, so the real VisionIOLimelight reads it unchanged.
        for (int i = 0; i < VisionConstants.CAMERA_NAMES.length; i++) {
          simWorld.add(
              "Camera" + i,
              new LimelightSim(VisionConstants.CAMERA_NAMES[i], driveSim::getTruePose));
        }
        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[0],
                    VisionConstants.ROBOT_TO_CAMERA[0],
                    drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[1],
                    VisionConstants.ROBOT_TO_CAMERA[1],
                    drive::getRotation));

        ElevatorSimulation elevatorSimulation = new ElevatorSimulation();
        simWorld.add("Elevator", elevatorSimulation);
        elevator = new Elevator(new ElevatorIOTalonFXSim(elevatorSimulation));

        simWorld.start();
      }
      default -> {
        simWorld = null;
        driveSim = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIO() {},
                new VisionIO() {});

        elevator = null;
      }
    }

    Controls.bind(drive, elevator);
    configureDashboardButtons();
    configureAutos();
  }

  private void configureDashboardButtons() {}

  private void configureAutos() {
    // Characterizations
    autoChooser.add("Drive Systems Check", DriveSystemsCheck.create(drive));
    autoChooser.add(
        "Drive FF Characterization", DriveCharacterization.feedforwardCharacterization(drive));
    autoChooser.add(
        "Drive Wheel Radius Characterization",
        DriveCharacterization.wheelRadiusCharacterization(drive));
    autoChooser.add("Square Tune Drivetrain", DriveCharacterization.squarePIDTune(drive));

    // Autons
    autoChooser.add("Drive Forward", Autos.driveForward(drive));
    autoChooser.add("None", Autos.none(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Measured from the wheels rather than the gyro, so it still works if the gyro drops out.
  private double yawVelocity() {
    return drive.getChassisVelocities().omega;
  }

  /** Call every loop before the scheduler runs, so commands see this cycle's inputs. */
  public void periodic() {
    drive.periodic();
    vision.periodic();
    elevator.periodic();

    if (driveSim != null) {
      Logger.recordOutput("Sim/TruePose", driveSim.getTruePose());
      simWorld.logOutputs();
    }
  }
}
