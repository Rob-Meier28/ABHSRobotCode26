// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final SparkMax lClimb;
  private final SparkMax rClimb;
  /** Creates a new Climber. */
  public Climber() {
    lClimb = new SparkMax(ClimberConstants.lClimbCanId, MotorType.kBrushless);
    rClimb = new SparkMax(ClimberConstants.rClimbCanId, MotorType.kBrushless);
  }

  public void Telescope(double speed, boolean extend) {
    if(extend == true) {
      lClimb.set(-1 * speed);
      rClimb.set(speed);
    } else {
      lClimb.set(speed);
      rClimb.set(-1 * speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
