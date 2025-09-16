package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class FieldCentricDrive extends CommandBase {

    private DriveSubsystem drive;
    private  GamepadEx driverJoystick;
    public FieldCentricDrive(DriveSubsystem drive, GamepadEx driverJoystick) {
        this.drive = drive;
        this.driverJoystick = driverJoystick;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double leftX = driverJoystick.getLeftX();
        double leftY = driverJoystick.getLeftY();
        double rightX = driverJoystick.getRightX();
        drive.fieldCentricDrive(leftX, leftY, rightX);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldCentricDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
