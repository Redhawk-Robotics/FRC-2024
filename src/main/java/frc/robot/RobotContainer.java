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
import frc.constants.Settings.ShooterConstants;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.commands.automation.IntakeToShooter;
import frc.robot.commands.automation.PivotToShoot;
import frc.robot.commands.automation.ShootNote;
import frc.robot.commands.automation.SourceIntake;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkFLEX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSparkMAX;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.shooter.ShooterWheelStates;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOPigeon;
import frc.robot.subsystems.swerve.ModuleIOSparkMAX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.NoteVisualizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Setting Up Autonomous Dropdown */

  SendableChooser<String> m_autonChooser = new SendableChooser<>();

  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  /* Driver Controls */
  private final int translationAxisLeft = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int translationAxisRight = XboxController.Axis.kRightY.value;

  /* DRIVER BUTTONS */
  private final Trigger zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  // private final JoystickButton robotCentric = new JoystickButton(DRIVER,
  // XboxController.Button.kLeftBumper.value);

  // Xbox triggers
  private final int Driver_leftTrigger = XboxController.Axis.kLeftTrigger.value;
  private final int Driver_rightTrigger = XboxController.Axis.kRightTrigger.value;

  // Xbox buttons
  private final Trigger Driver_XButton = new JoystickButton(DRIVER, XboxController.Button.kX.value);
  private final Trigger Driver_YButton = new JoystickButton(DRIVER, XboxController.Button.kY.value);
  private final Trigger Driver_BButton = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  // Xbox bumpers
  private final Trigger Driver_leftBumper =
      new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);
  private final Trigger dampen =
      new JoystickButton(DRIVER, XboxController.Button.kRightBumper.value);

  // Extra Buttons
  private final Trigger startButton =
      new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger backButton = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  // Dpad buttons
  private final Trigger Driver_up = new POVButton(DRIVER, 0);
  private final Trigger Driver_down = new POVButton(DRIVER, 180);
  private final Trigger Driver_right = new POVButton(DRIVER, 90);
  private final Trigger Driver_left = new POVButton(DRIVER, 270);

  /* OPERATOR BUTTONS */
  // Xbox values
  private final int OP_leftYAxis = XboxController.Axis.kLeftY.value;
  private final int OP_leftXAxis = XboxController.Axis.kLeftX.value;
  private final int OP_rightXAxis = XboxController.Axis.kRightX.value;
  private final int OP_rightYAxis = XboxController.Axis.kRightY.value;

  // Xbox triggers
  private final int OP_leftTrigger = XboxController.Axis.kLeftTrigger.value;
  private final int OP_rightTrigger = XboxController.Axis.kRightTrigger.value;
  // Xbox buttons
  private final Trigger OP_AButton = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger OP_XButton = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger OP_YButton = new JoystickButton(OPERATOR, XboxController.Button.kY.value);
  private final Trigger OP_BButton = new JoystickButton(OPERATOR, XboxController.Button.kB.value);

  // Xbox bumpers
  private final Trigger OP_rightBumper =
      new JoystickButton(OPERATOR, XboxController.Button.kRightBumper.value);
  private final Trigger OP_leftBumper =
      new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  // Extra Buttons
  private final Trigger OP_startButton =
      new JoystickButton(OPERATOR, XboxController.Button.kStart.value);
  private final Trigger OP_backButton =
      new JoystickButton(OPERATOR, XboxController.Button.kBack.value);

  // Dpad buttons
  private final Trigger OP_up = new Trigger(() -> OPERATOR.getPOV() == 0);
  private final Trigger OP_down = new Trigger(() -> OPERATOR.getPOV() == 180);
  private final Trigger OP_right = new Trigger(() -> OPERATOR.getPOV() == 90);
  private final Trigger OP_left = new Trigger(() -> OPERATOR.getPOV() == 270);
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
  private Climber climber;
  private ClimberIO climberIO;

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
        this.climberIO = new ClimberIOSparkFLEX();
        this.climber = new Climber(climberIO);
    }

    NoteVisualizer.setRobotPoseSupplier(swerve::getPathPlannerPose);
    setupNamedCommands();

    swerve.setDefaultCommand(
        new Drive(
            swerve,
            () -> -DRIVER.getRawAxis(translationAxisLeft),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> true,
            () -> dampen.getAsBoolean(),
            () -> 1 // speed
            // multiplier
            ));

    // shooter.setDefaultCommand(new ShooterWheels(shooter, swerve::getRobotPose));
    climber.setDefaultCommand(
        new frc.robot.commands.climber.Climber(
            climber,
            () -> -OPERATOR.getRawAxis(translationAxisLeft),
            () -> OPERATOR.getRawAxis(translationAxisRight)));

    // Configure the button bindings
    configureButtonBindings();
    configrueAutos();
  }

  private void configureButtonBindings() {
    /*
     * Driver Buttons
     */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    // Driver_right.toggleOnTrue(new SourceIntake(pivot, shooter));

    // Driver_XButton.toggleOnTrue(
    // new IntakeToShooter(intake, pivot, shooter)
    // .handleInterrupt(() -> CommandPreparer.prepareToStopIntakeToPivot())
    // .andThen(
    // new PivotToShoot(
    // pivot, PivotStates.kPivotSubwoofer, ShooterWheelStates.kShooterFullShot)));

    // Driver_BButton.toggleOnTrue(
    // new PivotToShoot(pivot, PivotStates.kPivotSubwoofer,
    // ShooterWheelStates.kShooterFullShot)
    // .andThen(
    // new ShootNote(shooter, pivot)
    // .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter())));

    // Driver_XButton.toggleOnTrue(
    // new InstantCommand(() -> Pivot.setPivotState(PivotStates.kPivotHome)));
    // Driver_BButton.toggleOnTrue(new InstantCommand(() ->
    // Pivot.setPivotState(PivotStates.kEject)));

    Driver_YButton.toggleOnTrue(shooter.rejectNote().alongWith(intake.enableReverseIntake()))
        .toggleOnFalse(shooter.stopNoteRejection().alongWith(intake.stopIntake()));

    // ! TO BE REPLACED WITH SWERVE X-LOCK
    // DRIVER_up.onTrue(new InstantCommand(() -> States.driveState =
    // States.DriveStates.d90))
    // .onFalse(new InstantCommand(() -> States.driveState =
    // States.DriveStates.standard));

    /*
     * OPERATOR CONTROLS
     */

    // ^Eject Note/horizontal shot
    OP_up.whileTrue(shooter.rejectNote().alongWith(intake.enableReverseIntake()))
        .toggleOnFalse(shooter.stopNoteRejection().alongWith(intake.stopIntake()));

    // ^Source Intake
    OP_down.toggleOnTrue(
        new SourceIntake(pivot, shooter)
            .handleInterrupt(() -> CommandPreparer.prepareToStopShooterAndPivot()));

    // ^Confirmed Shot
    OP_rightBumper.onTrue(new ShootNote(shooter, pivot, ShooterConstants.currentShooterState));

    // ^Pivot Home
    OP_leftBumper.toggleOnTrue(
        new InstantCommand(() -> Pivot.setPivotState(PivotStates.kPivotHome))
            .alongWith(
                new InstantCommand(
                    () -> Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle))));

    // ^Floor Intake
    OP_left.whileTrue(new IntakeToShooter(intake, pivot, shooter))
        .toggleOnFalse(new InstantCommand(() -> CommandPreparer.prepareToStopIntakeToPivot()));

    // ^Floor Intake Reverse
    OP_right.whileTrue(new InstantCommand(() -> Intake.setIntakeState(IntakeState.kIntakeReverse)))
        .toggleOnFalse(new InstantCommand(() -> CommandPreparer.prepareToStopIntakeToPivot()));

    // ^Subwoofer
    OP_AButton.toggleOnTrue(
        new PivotToShoot(pivot, PivotStates.kPivotSubwoofer, ShooterWheelStates.kShooterFullShot)
            .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter()));

    // ^Podium
    OP_BButton.toggleOnTrue(
            new PivotToShoot(
                    pivot, PivotStates.kPivotPodium, ShooterWheelStates.kShooterThreeQuarterShot)
                .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter()))
        .debounce(Driver_leftTrigger);

    // ^Center Line
    OP_XButton.toggleOnTrue(
        new PivotToShoot(pivot, PivotStates.kCenterLine, ShooterWheelStates.kShooterFullShot)
            .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter()));

    // ! Amp comand has to be made
    OP_YButton.toggleOnTrue(
        new PivotToShoot(pivot, PivotStates.kPivotAmp, ShooterWheelStates.kAmpShot)
            .handleInterrupt(() -> CommandPreparer.prepareToStopAllShooter()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("3P");
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

  public void setupNamedCommands() {
    NamedCommands.registerCommand(
        "Intake",
        new SequentialCommandGroup(
            Commands.print("[AUTO COMMAND] IntakeToPivot Command"),
            new IntakeToShooter(intake, pivot, shooter)));

    NamedCommands.registerCommand(
        "Pivot Subwoofer",
        new SequentialCommandGroup(
            Commands.print("[AUTO COMMAND] PivotToShoot subwoofer Command"),
            new PivotToShoot(pivot, PivotStates.kPivotSubwoofer)));

    NamedCommands.registerCommand(
        "Pivot Side",
        new SequentialCommandGroup(
            Commands.print("[AUTO COMMAND] PivotToShoot mid Command"),
            new PivotToShoot(pivot, PivotStates.kPivotNoteSides)));

    NamedCommands.registerCommand(
        "Pivot Mid",
        new SequentialCommandGroup(
            Commands.print("[AUTO COMMAND] PivotToShoot mid note Command"),
            new PivotToShoot(pivot, PivotStates.kPivotNoteMid)));

    NamedCommands.registerCommand(
        "Shoot",
        new SequentialCommandGroup(
            Commands.print("[AUTO COMMAND] ShootNote Command"), new ShootNote(shooter, pivot)));
  }

  public void configrueAutos() {
    m_autonChooser.setDefaultOption("3 Piece Auto", Settings.AutoPaths.k3P);
    m_autonChooser.setDefaultOption("Middle Auto 4", Settings.AutoPaths.b);

    SmartDashboard.putData(m_autonChooser);
  }
}
