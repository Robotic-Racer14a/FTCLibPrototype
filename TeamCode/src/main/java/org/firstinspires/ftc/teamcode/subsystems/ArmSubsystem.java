package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class ArmSubsystem extends SubsystemBase {

    private final Motor armMotor = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312);
    private final PIDController pid = new PIDController(0.001, 0, 0);
    private double targetPos = 0;

    public ArmSubsystem () {

    }

    @Override
    public void periodic () {

    }

    public double getArmPos () {
        return armMotor.getCurrentPosition();
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

}
