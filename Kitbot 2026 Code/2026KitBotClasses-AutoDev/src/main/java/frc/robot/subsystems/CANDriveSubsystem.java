// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix6.hardware.Pigeon2;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftLeaderEncoder;
  private final RelativeEncoder leftFollowerEncoder;
  private final RelativeEncoder rightLeaderEncoder;
  private final RelativeEncoder rightFollowerEncoder;

  private final SparkClosedLoopController PidLeftLeader;
  private final SparkClosedLoopController PidLeftFollower;
  private final SparkClosedLoopController PidRightLeader;
  private final SparkClosedLoopController PidRightFollower;

  private final DifferentialDrive drive;
  private final Pigeon2 m_gyro = new Pigeon2(9);

  public CANDriveSubsystem() {
    // create brushless motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    leftLeaderEncoder = leftLeader.getEncoder();
    leftFollowerEncoder = leftFollower.getEncoder();
    rightLeaderEncoder = rightLeader.getEncoder();
    rightFollowerEncoder = rightFollower.getEncoder();

    PidLeftLeader = leftLeader.getClosedLoopController();
    PidLeftFollower = leftLeader.getClosedLoopController();
    PidRightLeader = leftLeader.getClosedLoopController();
    PidRightFollower = leftLeader.getClosedLoopController();

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
    Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees());

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void zeroHeading() {
    m_gyro.reset();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()).getDegrees();
  }

  public double[] getPositionRotations() {
    double[] positions = new double[4];
    positions[0] = leftLeaderEncoder.getPosition();
    positions[1] = leftFollowerEncoder.getPosition();
    positions[2] = rightLeaderEncoder.getPosition();
    positions[3] = rightFollowerEncoder.getPosition();
    return positions;
  }

  public void setAutoDestination(int value, double destination) {
    if (value == 0) {
      PidLeftLeader.setSetpoint(destination, ControlType.kPosition);
    } else if (value == 1) {
      PidLeftFollower.setSetpoint(destination, ControlType.kPosition);
    } else if (value == 2) {
      PidRightLeader.setSetpoint(destination, ControlType.kPosition);
    } else if (value == 3) {
      PidRightFollower.setSetpoint(destination, ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
