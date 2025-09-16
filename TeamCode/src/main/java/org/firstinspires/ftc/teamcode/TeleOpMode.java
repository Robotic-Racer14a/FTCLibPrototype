package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class TeleOpMode extends OpMode {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final GamepadEx driverJoystick = new GamepadEx(gamepad1);
    private final GamepadEx operator = new GamepadEx(gamepad2);


    @Override
    public void init() {
        CommandScheduler.getInstance().enable();

        //Default Commands
        drive.setDefaultCommand(new FieldCentricDrive(drive, driverJoystick));

        //Driver Controls
        driverJoystick.getGamepadButton(GamepadKeys.Button.A).whenActive(new DriveToPose(drive));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
