// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.autos;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCommands;
import org.wpilib.command3.Command;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Add your docs here. */
public final class Autos {
  private Autos() {}

  /** Sits still. The safe default. */
  public static Command none(Drive drive) {
    return drive.run(coroutine -> drive.stop()).named("Auto.None");
  }

  public static Command driveForward(Drive drive) {
    return DriveCommands.driveFor(drive, new ChassisVelocities(1.0, 0.0, 0.0), 2.0);
  }
}
