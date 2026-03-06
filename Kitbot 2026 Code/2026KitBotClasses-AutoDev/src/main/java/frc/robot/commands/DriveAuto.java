// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.Constants.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAuto extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;

  private double[] positions = new double[4], destinations = new double[4];
  private double xDistance, xRotations;
  private boolean reverse;

  public DriveAuto(CANDriveSubsystem driveSystem, double xDistance, boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xDistance = xDistance;
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positions = driveSubsystem.getPositionRotations();

    xRotations = xDistance / DriveConstants.driveWheelCircumference * DriveConstants.driveGearRatio;

    if (reverse == false) {
      destinations[0] = positions[0] + xRotations;
      destinations[1] = positions[1] + xRotations;
      destinations[2] = positions[2] + xRotations;
      destinations[3] = positions[3] + xRotations;
    } else {
      destinations[0] = positions[0] - xRotations;
      destinations[1] = positions[1] - xRotations;
      destinations[2] = positions[2] - xRotations;
      destinations[3] = positions[3] - xRotations;
    }

    driveSubsystem.setAutoDestination(0, destinations[0]);
    driveSubsystem.setAutoDestination(2, destinations[1]);
    driveSubsystem.setAutoDestination(2, destinations[2]);
    driveSubsystem.setAutoDestination(3, destinations[3]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    positions = driveSubsystem.getPositionRotations();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(positions == destinations) {
      return true;
    } else {
      return false;
    }
  }
}
