package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOpMode extends OpMode {
    private DriveSubsystem drive = new DriveSubsystem();
    private GamepadEx driver = new GamepadEx(gamepad1);
    private GamepadEx operator = new GamepadEx(gamepad2);


    @Override
    public void init() {
        CommandScheduler.getInstance().run();
        drive.setDefaultCommand(new FieldCentricDrive(drive));
        driver.getGamepadButton(GamepadKeys.Button.A).whenActive(new DriveToPose(drive));
    }

    @Override
    public void loop() {

    }
}
