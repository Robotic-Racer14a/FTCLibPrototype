package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.ArmDefault;
import org.firstinspires.ftc.teamcode.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoExample extends OpMode {

    private final RobotClass robot = new RobotClass();

    @Override
    public void init() {
        //Schedule Auto
        CommandScheduler.getInstance().schedule(new AutoExampleCommand(robot));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}




class AutoExampleCommand extends SequentialCommandGroup {

    public AutoExampleCommand(RobotClass robot) {
        addCommands(
                new DriveToPose(robot.drive, new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
                new WaitCommand(500),
                new InstantCommand(() -> robot.arm.setTargetPos(500)),
                new WaitCommand(500),
                new DriveToPose(robot.drive, new Pose2d(-30, 30, Rotation2d.fromDegrees(0)))
        );

    }
}
