package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp
public class TeleOpMode extends OpMode {

//    private final RobotClass robot = new RobotClass(
//            new Motor(hardwareMap, "leftFront"),
//            new Motor(hardwareMap, "rightFront"),
//            new Motor(hardwareMap, "leftRear"),
//            new Motor(hardwareMap, "rightRear"),
//            new RevIMU(hardwareMap)
//
//    );

    private final DriveSubsystem drive = new DriveSubsystem(
            new Motor(hardwareMap, "leftFront"),
            new Motor(hardwareMap, "rightFront"),
            new Motor(hardwareMap, "leftRear"),
            new Motor(hardwareMap, "rightRear"),
            new RevIMU(hardwareMap)
    );

    private final GamepadEx driverJoystick = new GamepadEx(gamepad1);

    @Override
    public void init() {
        //Driver Controls
        //robot.driverJoystick.getGamepadButton(GamepadKeys.Button.A).whenActive(new DriveToPose(robot.drive, new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    @Override
    public void init_loop() {
        //robot.drive.updateRobotPoseMT1();
    }

    @Override
    public void loop() {
        //CommandScheduler.getInstance().run();
        drive.fieldCentricDrive(driverJoystick.getLeftX(), driverJoystick.getLeftY(), driverJoystick.getRightX());
    }
}
