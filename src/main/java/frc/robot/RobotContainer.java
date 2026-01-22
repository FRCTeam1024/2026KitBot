package frc.robot;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver =
      new CommandXboxController(ControlConstants.driverPort);

  private final CommandXboxController operator =
      new CommandXboxController(ControlConstants.operatorPort);
  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final FuelSubsystem fuelSubsystem = new FuelSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCmd(
            () -> applyDeadband(-driver.getLeftY(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getLeftX(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getRightX(), ControlConstants.stickDeadband)));

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* Driver Buttons */
    driver.y().onTrue(Commands.runOnce(() -> swerve.zeroHeading()).ignoringDisable(true));

    // While the left bumper on operator controller is held, intake Fuel
    operator
        .leftBumper()
        .whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.intake(), () -> fuelSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operator
        .rightBumper()
        .whileTrue(
            fuelSubsystem
                .spinUpCommand()
                .withTimeout(Constants.FuelConstants.SPIN_UP_SECONDS)
                .andThen(fuelSubsystem.launchCommand())
                .finallyDo(() -> fuelSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operator
        .a()
        .whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.eject(), () -> fuelSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
