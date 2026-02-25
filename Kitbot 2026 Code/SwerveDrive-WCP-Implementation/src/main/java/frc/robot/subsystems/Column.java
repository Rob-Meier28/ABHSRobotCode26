// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ColumnConstants;

public class Column extends SubsystemBase {
  private final SparkFlex shooter;
  private final SparkFlex feeder;

  /** Creates a new Column. */
  public Column() {
    shooter = new SparkFlex(ColumnConstants.shooterMotorCanId, MotorType.kBrushless);
    feeder = new SparkFlex(ColumnConstants.feederMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
