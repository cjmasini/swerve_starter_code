package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Move elevator to a specific position
 */
public class ScoreCoralCommand extends SequentialCommandGroup {

    /**
     * Command for setting the elevator to a specific position
     *
     * @param targetLevel
     *                          The target level to move the elevator to.
     * @param elevatorSubsystem
     *                          The elevator subsystem.
     */
    public ScoreCoralCommand(SendableChooser<Level> levelChooser, ElevatorSubsystem elevatorSubsystem,
            CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            ReefPosition side) {
        addRequirements(elevatorSubsystem, coralSubsystem);

        Command driveToReefCommand = new DriveToReefCommand(driveSubsystem, visionSubsystem, side);
        Command raiseElevator = new AutoSetElevatorLevelCommand(levelChooser, elevatorSubsystem);
        Command scoreCoral = new EjectCoralCommand(coralSubsystem);
        Command waitCommand = new WaitCommand(1);
        Command lowerElevator = new SetElevatorLevelCommand(Level.DOWN, elevatorSubsystem);
        this.addCommands(raiseElevator, scoreCoral, waitCommand, lowerElevator);
        // this.addCommands(driveToReefCommand, raiseElevator, scoreCoral, waitCommand, lowerElevator);

    }

    public ScoreCoralCommand(Level targetLevel, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        addRequirements(elevatorSubsystem, coralSubsystem);

        Command raiseElevator = new SetElevatorLevelCommand(targetLevel, elevatorSubsystem);
        Command scoreCoral = new EjectCoralCommand(coralSubsystem);
        Command waitCommand = new WaitCommand(1);
        Command lowerElevator = new SetElevatorLevelCommand(Level.DOWN, elevatorSubsystem);
        this.addCommands(raiseElevator, scoreCoral, waitCommand, lowerElevator);
    }
}
