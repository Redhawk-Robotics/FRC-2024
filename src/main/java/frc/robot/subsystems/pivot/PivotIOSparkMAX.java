package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.constants.Ports;
import frc.constants.Settings;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotIOSparkMAX implements PivotIO {
    private final CANSparkMax leftPivot, rightPivot;
    private SparkPIDController pivotController;
    private AbsoluteEncoder pivtotEncoder;

    public PivotIOSparkMAX() {
        /*
         * Sparkmax
         */
        this.leftPivot = new CANSparkMax(Ports.pivotID.leftPivot, MotorType.kBrushless);
        this.rightPivot = new CANSparkMax(Ports.pivotID.rightPivot, MotorType.kBrushless);

        leftPivot.restoreFactoryDefaults();
        rightPivot.restoreFactoryDefaults();

        this.leftPivot.setInverted(Settings.Pivot.leftPivotInvert);
        this.rightPivot.setInverted(Settings.Pivot.rightPivotInvert);

        this.rightPivot.follow(leftPivot);

        this.leftPivot.setIdleMode(Settings.Pivot.pivotNeutralMode);
        this.rightPivot.setIdleMode(Settings.Pivot.pivotNeutralMode);

        this.leftPivot.setSmartCurrentLimit(Settings.Pivot.armContinousCurrentLimit);
        this.rightPivot.setSmartCurrentLimit(Settings.Pivot.armContinousCurrentLimit);

        this.leftPivot.enableVoltageCompensation(Settings.Pivot.maxVoltage);
        this.rightPivot.enableVoltageCompensation(Settings.Pivot.maxVoltage);

        this.pivotController = leftPivot.getPIDController();
        this.pivotController.setFeedbackDevice(pivtotEncoder);

        this.pivotController.setP(Settings.Pivot.pivotKP);
        this.pivotController.setI(Settings.Pivot.pivotKI);
        this.pivotController.setD(Settings.Pivot.pivotKD);
        this.pivotController.setFF(Settings.Pivot.pivotKFF);

        this.pivotController.setOutputRange(Settings.Pivot.MIN_INPUT, Settings.Pivot.MAX_INPUT);

        this.pivtotEncoder = leftPivot.getAbsoluteEncoder(Type.kDutyCycle);
        this.pivtotEncoder.setInverted(false);
        this.pivtotEncoder.setZeroOffset(Settings.Pivot.ZERO_OFFSET);// FIXME need to find the value

        this.leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        this.leftPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        this.leftPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0); // TODO check the value for both forward and                                                                            // TODO reverse
        this.leftPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        this.rightPivot.burnFlash();
        this.leftPivot.burnFlash();

        SmartDashboard.putBoolean("Forward Soft Limit",
                leftPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));

        SmartDashboard.putBoolean("Reverse Soft Limit",
                leftPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
    }

    /*
     *
     *
     * Overriden Interface methods
     */
    @Override
    public void updateInputs(PivotIOInputs inputs) {
    }

    @Override
    public void pivotStop() {
        setMotorSpeeds(0);
    }

    @Override
    public void pivotUp() {
        setMotorSpeeds(1);
    }

    @Override
    public void pivotDown() {
        setMotorSpeeds(-1);
    }

    @Override
    public void pivotApplySpeed(double speed) {
        setMotorSpeeds(speed);
    }

    /*
     *
     * Class methods
     *
     */
    public void setMotorSpeeds(double speed) {
        leftPivot.set(speed);
        rightPivot.set(-speed);
    }

    public double getPosition() {
        return pivtotEncoder.getPosition();
    }
}
