package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkFlex rollers;
    private final SparkMax arm;

    public Intake() {
        rollers = new SparkFlex(IntakeConstants.rollersMotorCanId, MotorType.kBrushless);
        arm = new SparkMax(IntakeConstants.armMotorCanId, MotorType.kBrushless);
    }

}
