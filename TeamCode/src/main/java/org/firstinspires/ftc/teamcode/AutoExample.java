package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.ArmDefault;
import org.firstinspires.ftc.teamcode.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous
public class AutoExample extends OpMode {

    private final RobotClass robot = new RobotClass(
            new Motor(hardwareMap, "leftFront"),
            new Motor(hardwareMap, "rightFront"),
            new Motor(hardwareMap, "leftRear"),
            new Motor(hardwareMap, "rightRear"),
            new RevIMU(hardwareMap)

    );

    @Override
    public void init() {
        //Schedule Auto
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new DriveToPose(robot.drive, new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
                new WaitCommand(500),
//                new InstantCommand(() -> robot.arm.setTargetPos(500)),
//                new WaitCommand(500),
                new DriveToPose(robot.drive, new Pose2d(-30, 30, Rotation2d.fromDegrees(0)))
        ));
    }

    @Override
    public void init_loop() {
        robot.drive.updateRobotPoseMT1();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        robot.autoEndBlackboardCode();
    }
}

