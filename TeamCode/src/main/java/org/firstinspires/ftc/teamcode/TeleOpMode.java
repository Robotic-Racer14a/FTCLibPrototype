package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDefault;
import org.firstinspires.ftc.teamcode.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class TeleOpMode extends OpMode {

    private final RobotClass robot = new RobotClass();

    @Override
    public void init() {
        //Driver Controls
        robot.driverJoystick.getGamepadButton(GamepadKeys.Button.A).whenActive(new DriveToPose(robot.drive, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    @Override
    public void init_loop() {
        robot.drive.updateRobotPoseMT1();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
