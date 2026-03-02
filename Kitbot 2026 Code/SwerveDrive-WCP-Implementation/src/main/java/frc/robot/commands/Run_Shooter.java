
package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Run_Shooter extends Command {

    private final double IndexerSpeed;
    private final double ShooterSpeed;
    private final Shooter Shooter;

    public Run_Shooter(Shooter shooter, double IndexerSpeed, double ShooterSpeed) {
        this.Shooter = shooter;
        this.IndexerSpeed = IndexerSpeed;
        this.ShooterSpeed = ShooterSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
  
    }

    @Override
    public void execute() {
        Shooter.indexer.set(IndexerSpeed);
        Shooter.shooter.set(ShooterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.indexer.set(0);
        Shooter.shooter.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
