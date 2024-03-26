package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.SwerveConfig.REV;
import frc.robot.commands.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.ModuleIOSparkMAX;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final XboxController DRIVER = new XboxController(0);
  // private final XboxController OPERATOR = new XboxController(1);

  /* Driver Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  // private final JoystickButton robotCentric = new JoystickButton(DRIVER,
  // XboxController.Button.kLeftBumper.value);

  private final JoystickButton dampen = new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);

  private final POVButton up = new POVButton(DRIVER, 90);
  private final POVButton down = new POVButton(DRIVER, 270);
  private final POVButton right = new POVButton(DRIVER, 180);
  private final POVButton left = new POVButton(DRIVER, 0);

  /* Subsystems */
  private Swerve swerve;
  private GyroIO gyroIO;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // ! Currently ONLY run Robot on REAL
    switch (Constants.currentMode) {
      case SIM:
        // ! CAUTION CAUTION
        this.gyroIO = new GyroIONavX();
        this.swerve = new Swerve(gyroIO, null, null, null, null);
        break;
      case REPLAY:
        // ! CAUTION CAUTION
        this.gyroIO = new GyroIONavX();
        this.swerve = new Swerve(gyroIO, null, null, null, null);
        break;
      default:
        this.gyroIO = new GyroIONavX();
        this.swerve = new Swerve(
            gyroIO,
            new ModuleIOSparkMAX(0, REV.revSwerveModuleConstants[0]),
            new ModuleIOSparkMAX(1, REV.revSwerveModuleConstants[1]),
            new ModuleIOSparkMAX(2, REV.revSwerveModuleConstants[2]),
            new ModuleIOSparkMAX(3, REV.revSwerveModuleConstants[3]));
    }

    swerve.setDefaultCommand(
        new Drive(
            swerve,
            () -> -DRIVER.getRawAxis(translationAxis),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> true,
            () -> dampen.getAsBoolean(),
            () -> 1 // speed multiplier
        ));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // heading lock bindings
    up.onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d90))
        .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
    left.onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d180))
        .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
    right
        .onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d0))
        .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
    down.onTrue(new InstantCommand(() -> States.driveState = States.DriveStates.d270))
        .onFalse(new InstantCommand(() -> States.driveState = States.DriveStates.standard));
  }

  public Command getAutonomousCommand() {
    var path = swerve.followPathCommand("TEST");
    return new FunctionalCommand(
        path::initialize,
        path::execute,
        new Consumer<Boolean>() {
          public void accept(Boolean e) {
            System.out.println("Auto has ended");
          }
        },
        new BooleanSupplier() {
          public boolean getAsBoolean() {
            return !DriverStation.isAutonomousEnabled();
          }
        },
        swerve);
  }
}
