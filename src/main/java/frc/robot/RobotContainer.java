// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.AutoSetElevatorLevelCommand;
import frc.robot.commands.AutonomousCommandFactory;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.EjectCoralCommand;
import frc.robot.commands.FallCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain, visionSubsystem,
      elevatorSubsystem, coralSubsystem);

  private SendableChooser<Level> levelChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    MoveCommand moveCommand = new MoveCommand(this.drivetrain, driverXbox);
    drivetrain.setDefaultCommand(moveCommand);

    configureBindings();

    SmartDashboard.putData(levelChooser);
    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    // Eject coral onto reef
    EjectCoralCommand ejectCoralCommand = new EjectCoralCommand(coralSubsystem);
    driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(ejectCoralCommand.withTimeout(10));

    // Climb the cage while button is pressed
    ClimbCommand climbCommand = new ClimbCommand(climbSubsystem);
    driverXbox.a().and(driverXbox.rightTrigger().negate()).whileTrue(climbCommand);

    // Slowly lower after climbing the cage
    FallCommand fallCommand = new FallCommand(climbSubsystem);
    driverXbox.b().and(driverXbox.rightTrigger().negate()).whileTrue(fallCommand);

    // Intake coral on left bumper press
    IntakeCommand intakeCommand = new IntakeCommand(elevatorSubsystem, coralSubsystem, intakeSubsystem);
    driverXbox.leftTrigger().and(driverXbox.rightTrigger().negate()).onTrue(intakeCommand);
    
    // Set the elevator to a specific position
    for (Level level : Level.values()) {
      levelChooser.addOption(level.toString(), level);
    }
    levelChooser.setDefaultOption("DOWN", Level.DOWN);
    ScoreCoralCommand scoreLeftCoralCommand = new ScoreCoralCommand(levelChooser, elevatorSubsystem, coralSubsystem, drivetrain, visionSubsystem, ReefPosition.LEFT);
    // AutoSetElevatorLevelCommand scoreCoralCommand = new
    // AutoSetElevatorLevelCommand(levelChooser, elevatorSubsystem);
    driverXbox.leftBumper().onTrue(scoreLeftCoralCommand);

    // TODO: Switch to score command when ready
    // ScoreCoralCommand scoreRightCoralCommand = new ScoreCoralCommand(levelChooser, elevatorSubsystem, coralSubsystem, drivetrain, visionSubsystem, ReefPosition.LEFT);
    AutoSetElevatorLevelCommand scoreRightCoralCommand = new
      AutoSetElevatorLevelCommand(levelChooser, elevatorSubsystem);
    driverXbox.rightBumper().onTrue(scoreRightCoralCommand);

    // Right trigger is used to cancel other commands and as a modifier for face
    // buttons
    CancelCommand cancelCommand = new CancelCommand(
        List.of(coralSubsystem, climbSubsystem, elevatorSubsystem));
    driverXbox.rightTrigger().onTrue(cancelCommand.withTimeout(10));

    // Manually re-zero the gyro if it gets off during competition
    // With the pigeon Gyro, we only needed to do this because of user error in
    // setup
    InstantCommand resetGyro = new InstantCommand(() -> this.drivetrain.zeroHeading());
    driverXbox.rightStick().and(driverXbox.leftStick()).onTrue(resetGyro);

    // Toggle drive mode command, currently disabled as we did not find it necessary
    // InstantCommand toggleDriveMode = new InstantCommand(() ->
    // moveCommand.toggleFieldReletive());
    // driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(toggleDriveMode);
  }

  // Return the auto selected in smart dashboard
  public Command getAutonomousCommand() {
    if (SmartDashboard.getBoolean("Use Custom Auto?", false)) {
      return autoFactory.createAutoCommand(SmartDashboard.getString("Custom Auto", "Error"));
    } else {
      return autoChooser.getSelected();
    }
  }
}