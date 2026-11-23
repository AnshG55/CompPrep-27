package first.robot;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCharacterization;
import first.robot.subsystems.drive.DriveCommands;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.units.Units;

/**
 * Connects the driver's gamepad to commands. Bindings use CommandGamepad's positional names, so
 * they work with any gamepad the Driver Station recognizes. The DS maps every controller onto the
 * same layout:
 *
 * <pre>
 * CommandGamepad   Xbox      DualSense
 * faceDown         A         Cross
 * faceRight        B         Circle
 * faceLeft         X         Square
 * faceUp           Y         Triangle
 * left/rightBumper LB/RB     L1/R1
 * left/rightTrigger LT/RT    L2/R2
 * back / start     View/Menu Create/Options
 * </pre>
 *
 * CommandXboxController and CommandDualSenseController read these same buttons under brand names.
 * They add no buttons CommandGamepad lacks (it has the touchpad and misc buttons too), so choosing
 * one is only about which names read better.
 */
public final class Controls {
  private Controls() {}

  public static enum controlMode {
    START,
    BACK
  }

  public static controlMode currentmode = controlMode.BACK; // Default Starts as Back

  public static void bind(Drive drive) {
    CommandGamepad driver = new CommandGamepad(0);
    CommandGamepad operator = new CommandGamepad(1);

    // Bind Bindchoosing Buttons
    driver
        .back()
        .onTrue(
            Command.noRequirements(
                    coroutine -> {
                      currentmode = controlMode.BACK;
                    })
                .named("Bind Back"));

    driver
        .start()
        .onTrue(
            Command.noRequirements(
                    coroutine -> {
                      currentmode = controlMode.START;
                    })
                .named("Bind Start"));

    // Bind static controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    driver.faceDown().whileTrue(DriveCharacterization.squarePIDTune(drive));

    // Bind Back commands

    // Bind Start commands

    // Example:
    // driver
    //     .faceLeft()
    //     .and(new Trigger(() -> currentmode == controlMode.BACK))   //Use trigger with Control
    // Mode Enum
    //     .whileTrue(DriveCommands.driveToPose(drive, new Pose2d(0, 0, new Rotation2d(0))));
  }

  private static Command rumble(CommandGamepad driver, double seconds) {
    return Command.noRequirements(
            coroutine -> {
              setRumble(driver, 1.0);
              coroutine.wait(Units.Seconds.of(seconds));
              setRumble(driver, 0.0);
            })
        // Without this, canceling mid-rumble would leave the controller buzzing.
        .whenCanceled(() -> setRumble(driver, 0.0))
        .named("Driver.Rumble");
  }

  private static void setRumble(CommandGamepad driver, double strength) {
    driver.getHID().setRumble(RumbleType.LEFT_RUMBLE, strength);
    driver.getHID().setRumble(RumbleType.RIGHT_RUMBLE, strength);
  }
}
