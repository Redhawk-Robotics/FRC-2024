package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Ports;
import frc.constants.Ports.REV;
import frc.constants.Settings;
import frc.robot.commands.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.ModuleIOSparkMAX;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  /* Driver Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* DRIVER BUTTONS */
  private final Trigger zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  // private final JoystickButton robotCentric = new JoystickButton(DRIVER,
  // XboxController.Button.kLeftBumper.value);

  // Xbox triggers
  private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger = XboxController.Axis.kRightTrigger.value;

  // Xbox buttons
  private final Trigger XButton = new JoystickButton(DRIVER, XboxController.Button.kX.value);
  private final Trigger YButton = new JoystickButton(DRIVER, XboxController.Button.kY.value);
  private final Trigger BButton = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  // Xbox bumpers
  private final Trigger leftBumper =
      new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);
  private final Trigger dampen =
      new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);

  // Extra Buttons
  private final Trigger startButton =
      new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger backButton = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  // Dpad buttons
  private final Trigger up = new POVButton(DRIVER, 90);
  private final Trigger down = new POVButton(DRIVER, 270);
  private final Trigger right = new POVButton(DRIVER, 180);
  private final Trigger left = new POVButton(DRIVER, 0);

  /* OPERATOR BUTTONS */
  // Xbox values
  private final int OPleftYAxis = XboxController.Axis.kLeftY.value;
  private final int OPleftXAxis = XboxController.Axis.kLeftX.value;
  private final int OPrightXAxis = XboxController.Axis.kRightX.value;
  private final int OPrightYAxis = XboxController.Axis.kRightY.value;

  // Xbox triggers
  private final int OPleftTrigger = XboxController.Axis.kLeftTrigger.value;
  private final int OPrightTrigger = XboxController.Axis.kRightTrigger.value;
  // Xbox buttons
  private final Trigger OPAButton = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger OPXButton = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger OPYButton = new JoystickButton(OPERATOR, XboxController.Button.kY.value);
  private final Trigger OPBButton = new JoystickButton(OPERATOR, XboxController.Button.kB.value);

  // Xbox bumpers
  private final Trigger OPrightBumper =
      new JoystickButton(OPERATOR, XboxController.Button.kRightBumper.value);
  private final Trigger OPleftBumper =
      new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  // Extra Buttons
  private final Trigger OPstartButton =
      new JoystickButton(OPERATOR, XboxController.Button.kStart.value);
  private final Trigger OPbackButton =
      new JoystickButton(OPERATOR, XboxController.Button.kBack.value);

  // Dpad buttons
  private final Trigger OPup = new Trigger(() -> OPERATOR.getPOV() == 0);
  private final Trigger OPright = new Trigger(() -> OPERATOR.getPOV() == 90);
  private final Trigger OPDown = new Trigger(() -> OPERATOR.getPOV() == 180);
  private final Trigger OPLeft = new Trigger(() -> OPERATOR.getPOV() == 270);
  // END OF BUTTONS

  /* Subsystems */
  private Swerve swerve;
  private GyroIO gyroIO;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // ! Currently ONLY run Robot on REAL
    switch (Settings.currentMode) {
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
        this.gyroIO = new GyroIOPigeon();
        this.swerve =
            new Swerve(
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
            () -> 1 // speed
            // multiplier
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
    var path = swerve.followPathCommand("T1");
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
