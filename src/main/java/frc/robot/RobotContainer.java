package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.Ports;
import frc.constants.Ports.REV;
import frc.constants.Settings;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.commands.automation.IntakeToShooter;
import frc.robot.commands.automation.ShootNote;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSparkMAX;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.ModuleIOSparkMAX;
import frc.robot.subsystems.swerve.States;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.NoteVisualizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Configuring Shuffleboard */
    //private final SendableChooser<String> m_pathChooser = new SendableChooser<>();
    private final SendableChooser<String> m_autonChooser = new SendableChooser<>();

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
  private Shooter shooter;
  private ShooterIO shooterIO;
  private Pivot pivot;
  private PivotIO pivotIO;
  private Intake intake;
  private IntakeIO intakeIO;

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
        this.shooterIO = new ShooterIOSparkMAX();
        this.shooter = new Shooter(shooterIO);
        this.pivotIO = new PivotIOSparkMAX();
        this.pivot = new Pivot(pivotIO);
        this.intakeIO = new IntakeIOSparkMAX();
        this.intake = new Intake(intakeIO);
    }

    NoteVisualizer.setRobotPoseSupplier(swerve::getPathPlannerPose);

    NamedCommands.registerCommand(
        "Intake",
        new SequentialCommandGroup(
            Commands.print("We intaking"), new IntakeToShooter(intake, pivot, shooter)));
    NamedCommands.registerCommand(
        "Shoot",
        new SequentialCommandGroup(
            Commands.print("We shooting!"),
            new ShootNote(shooter, pivot, PivotStates.kPivotSubwoofer)));

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

    // shooter.setDefaultCommand(new ShooterWheels(shooter, swerve::getRobotPose));

    // Configure the button bindings
    setAutos();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    XButton.toggleOnTrue(
        new IntakeToShooter(intake, pivot, shooter)
            .handleInterrupt(() -> CommandPreparer.prepareForStoppingIntakeToPivot()));
    BButton.onTrue(
        new ShootNote(shooter, pivot, PivotStates.kPivotSubwoofer)
            .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter()));
    // BButton.onTrue(shooter.fullSpeedShooterWheels());

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

    /*
     * OPERATOR CONTROLS
     */
    // Pivot Manual Controls
    OPLeft.whileTrue(new InstantCommand(() -> pivot.pivotUp()));
    OPLeft.whileFalse(new InstantCommand(() -> pivot.pivotStop()));

    OPright.whileTrue(new InstantCommand(() -> pivot.pivotDown()));
    OPLeft.whileFalse(new InstantCommand(() -> pivot.pivotStop()));
  }

  public void setAutos() {
    m_autonChooser.setDefaultOption("Middle Auto 3", Settings.AutoConstants.kMiddleAuto3);
    SmartDashboard.putData("Autonomous", m_autonChooser);
  }
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(m_autonChooser.getSelected());
    // var path = swerve.followPathCommand("T1");
    // return new FunctionalCommand(
    // path::initialize,
    // path::execute,
    // new Consumer<Boolean>() {
    // public void accept(Boolean e) {
    // System.out.println("Auto has ended");
    // }
    // },
    // new BooleanSupplier() {
    // public boolean getAsBoolean() {
    // return !DriverStation.isAutonomousEnabled();
    // }
    // },
    // swerve);
  }
}
