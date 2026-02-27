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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  private final SparkMax arm;
  private final SparkFlex rollers;

  private final AbsoluteEncoder armEncoder;

  private final SparkClosedLoopController armPid;

  /** Creates a new Intake. */
  public IntakeSub() {
    arm = new SparkMax(IntakeConstants.armMotorCanId, MotorType.kBrushless);
    rollers = new SparkFlex(IntakeConstants.rollersMotorCanId, MotorType.kBrushless);

    armEncoder = arm.getAbsoluteEncoder();

    armPid = arm.getClosedLoopController();
    
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(IntakeConstants.PIDConstants.kP);
  }

  public void rotateArmTo(double degrees) {
    double rotations = degrees / 360 * IntakeConstants.armGearRatio;
    armPid.setSetpoint(rotations, ControlType.kPosition);
  }

  public void setIntake(double speed) {
    rollers.set(speed);
  }

  public double armAngle() {
    return armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
