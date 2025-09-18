package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.commands.ArmDefault;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class RobotClass {

    public final DriveSubsystem drive;
    //public final ArmSubsystem arm = new ArmSubsystem();
    public final GamepadEx driverJoystick = new GamepadEx(gamepad1);
    public final GamepadEx operator = new GamepadEx(gamepad2);

    public RobotClass (
            Motor leftFront,
            Motor rightFront,
            Motor leftRear,
            Motor rightRear,
            RevIMU imu
    ) {
        CommandScheduler.getInstance().enable();

        drive = new DriveSubsystem(
                leftFront,
                rightFront,
                leftRear,
                rightRear,
                imu);
        //Default Commands
        drive.setDefaultCommand(new FieldCentricDrive(drive, driverJoystick));
        //arm.setDefaultCommand(new ArmDefault(arm));

        initBlackboardCode();
    }

    public void initBlackboardCode() {
        try {
            if (blackboard.containsKey("Auto End Pose"))
                drive.setOdometry((Pose2d) blackboard.get("Auto End Pose"));
//            if (blackboard.containsKey("Auto End Arm Pos"))
//                arm.setArmPos((double) blackboard.get("Auto End Arm Pos"));
        } finally {
            telemetry.addLine("Unable to load position from auto");
        }
    }

    public void autoEndBlackboardCode () {
        blackboard.put("Auto End Pose", drive.getCurrentPose());
        //blackboard.put("Auto End Arm Pos", arm.getArmPos());
    }
}
