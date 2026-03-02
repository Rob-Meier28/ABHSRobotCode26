// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Shooter extends SubsystemBase {
 // private final SparkMax arm;
  //private final SparkFlex rollers;
  public final SparkFlex indexer;
  public final SparkFlex shooter;

  /** Creates a new Shooter. */
  public Shooter() {
    indexer = new SparkFlex(67, MotorType.kBrushless);
    shooter = new SparkFlex(68, MotorType.kBrushless);
    //SparkFlexConfig config = new SparkFlexConfig();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
