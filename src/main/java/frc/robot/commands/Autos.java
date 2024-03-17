// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveSubsystem subsystem) {
    return Commands.sequence();
  }

  public static Command getSimpleShootCommand(ShooterSubsystem shooterSubsystem) {
    return new InstantCommand(() -> {
      shooterSubsystem.fire(Constants.FLYWHEEL_FIRING_SPEED);
    }, shooterSubsystem);
  }

  public static Command Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
