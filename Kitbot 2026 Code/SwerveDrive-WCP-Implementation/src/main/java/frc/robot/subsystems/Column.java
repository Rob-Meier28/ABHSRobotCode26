package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ColumnConstants;

public class Column extends SubsystemBase {
    private final SparkFlex shooter;
    private final SparkFlex feeder;

    public Column() {
        shooter = new SparkFlex(ColumnConstants.shooterMotorCanId, MotorType.kBrushless);
        feeder = new SparkFlex(ColumnConstants.feederMotorCanId, MotorType.kBrushless);
    }
    
}
