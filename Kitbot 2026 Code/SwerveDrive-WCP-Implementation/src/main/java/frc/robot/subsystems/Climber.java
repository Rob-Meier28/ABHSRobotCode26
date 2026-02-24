package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final SparkMax lClimb;
    private final SparkMax rClimb;

    public Climber() {
        lClimb = new SparkMax(ClimberConstants.lClimbMotorCanId, MotorType.kBrushless);
        rClimb = new SparkMax(ClimberConstants.rClimbMotorCanId, MotorType.kBrushless);
    }
}
