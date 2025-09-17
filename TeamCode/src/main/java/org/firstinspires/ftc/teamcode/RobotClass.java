package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.ArmDefault;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class RobotClass {

    public final DriveSubsystem drive = new DriveSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    public final GamepadEx driverJoystick = new GamepadEx(gamepad1);
    public final GamepadEx operator = new GamepadEx(gamepad2);

    public RobotClass () {
        CommandScheduler.getInstance().enable();

        //Default Commands
        drive.setDefaultCommand(new FieldCentricDrive(drive, driverJoystick));
        arm.setDefaultCommand(new ArmDefault(arm));
    }
}
