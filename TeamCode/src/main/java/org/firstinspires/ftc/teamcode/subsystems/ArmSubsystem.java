package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmSubsystem extends SubsystemBase {

    private final MotorEx armMotor = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_312);
    private final PIDController pid = new PIDController(0.001, 0, 0);
    private double targetPos = 0;
    private double autoEndPosition = 0;

    public ArmSubsystem () {

    }

    @Override
    public void periodic () {

    }

    public double getArmPos () {
        return armMotor.getCurrentPosition() + autoEndPosition;
    }

    public void setArmPower (double power) {
        armMotor.set(power);
    }

    public void setTargetPos (double targetPos) {
        this.targetPos = targetPos;
    }

    public void runPID () {
        setArmPower(pid.calculate(getArmPos(), targetPos));
    }

    public void setArmPos (double pos) {
        armMotor.resetEncoder();
        autoEndPosition = pos;
    }

}
