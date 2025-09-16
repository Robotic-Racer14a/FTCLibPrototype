package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOpMode extends OpMode {
    private DriveSubsystem drive = new DriveSubsystem();
    private GamepadEx driverJoystick = new GamepadEx(gamepad1);
    private GamepadEx operator = new GamepadEx(gamepad2);


    @Override
    public void init() {
        CommandScheduler.getInstance().enable();
        drive.setDefaultCommand(new FieldCentricDrive(drive, driverJoystick));
        driverJoystick.getGamepadButton(GamepadKeys.Button.A).whenActive(new DriveToPose(drive));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

    }
}
