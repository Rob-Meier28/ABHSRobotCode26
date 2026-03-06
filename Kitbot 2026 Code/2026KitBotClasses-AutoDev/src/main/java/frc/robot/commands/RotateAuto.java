// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateAuto extends Command {
  /** Creates a new RotateAuto. */
  CANDriveSubsystem driveSubsystem;

  private double degrees, position, target;
  private boolean left;

  public RotateAuto(CANDriveSubsystem driveSystem, double degrees, boolean left) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    this.driveSubsystem = driveSystem;
    this.degrees = degrees;
    this.left = left;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = driveSubsystem.getHeading();
    
    if (left == false) {
      target = position + degrees;
    } else {
      target = position - degrees;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (left == false) {
      driveSubsystem.driveArcade(0, OperatorConstants.ROTATION_SCALING);
    } else {
     driveSubsystem.driveArcade(0, OperatorConstants.ROTATION_SCALING * -1); 
    }
    position = driveSubsystem.getHeading();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (target == position) {
      return true;
    } else {
      return false;
    }
  }
}
