// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  private final SparkMax arm;
  private final SparkFlex rollers;

  /** Creates a new Intake. */
  public IntakeSub() {
    arm = new SparkMax(IntakeConstants.armMotorCanId, MotorType.kBrushless);
    rollers = new SparkFlex(IntakeConstants.rollersMotorCanId, MotorType.kBrushless);
  }

  public void rotateArm(double speed) {
    arm.set(speed);
  }

  public void setIntake(double speed) {
    rollers.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
