package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmDefault extends CommandBase {

    private final ArmSubsystem arm;

    public ArmDefault (ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.runPID();
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
