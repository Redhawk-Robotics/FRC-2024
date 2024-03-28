package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Settings;

public class PivotIOSparkMAX implements PivotIO{
    private final CANSparkMax leftPivot, rightPivot;


public PivotIOSparkMAX(){
/*
     * Sparkmax
     */
    this.leftPivot = new CANSparkMax(Constants.pivotID.leftPivot, MotorType.kBrushless);
     leftPivot.setInverted(Settings.Pivot.leftPivotInvert);
    leftPivot.setIdleMode(Settings.Pivot.pivotNeutralMode);

    rightPivot = new CANSparkMax(Constants.pivotID.rightPivot, MotorType.kBrushless);
    rightPivot.setInverted(Settings.Pivot.rightPivotInvert);
    rightPivot.setIdleMode(Settings.Pivot.pivotNeutralMode);
    
}
}
