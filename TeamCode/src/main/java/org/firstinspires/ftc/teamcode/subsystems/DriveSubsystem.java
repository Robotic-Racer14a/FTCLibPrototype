package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class DriveSubsystem extends SubsystemBase {

    //Constants
    public static double TICKS_PER_REV = 383.6;
    public static double WHEEL_DIAMETER = 0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
    //TODO: Check units
    public static double TRACK_WIDTH = 6.75;

    public static double CENTER_WHEEL_OFFSET = 6;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;

    public static double B = 2.0;
    public static double ZETA = 0.7;



    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftRear;
    private final Motor rightRear;

    private final RevIMU imu;

    private MecanumDrive drive;

    private HolonomicOdometry odometry;

    private Limelight3A limelight;


    public DriveSubsystem (
            Motor leftFront,
            Motor rightFront,
            Motor leftRear,
            Motor rightRear,
            RevIMU imu
    ) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.imu = imu;

        drive= new MecanumDrive(false,
                leftFront, rightFront, leftRear, rightRear
        );

        odometry = new HolonomicOdometry(leftFront::getCurrentPosition, rightFront::getCurrentPosition, leftRear::getCurrentPosition, TRACK_WIDTH, CENTER_WHEEL_OFFSET);

        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setInverted(false);
        rightRear.setInverted(false);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        imu.init();
    }

    @Override
    public void periodic() {
        odometry.updatePose();
        updateRobotPoseMT2();
    }

    public Pose2d getCurrentPose() {
        return odometry.getPose();
    }

    public void fieldCentricDrive(double leftX, double leftY, double rightX) {
        drive.driveFieldCentric(
                leftX,
                leftY,
                rightX,
                imu.getRotation2d().getDegrees()
        );
    }

    public void robotCentricDrive(double leftX, double leftY, double rightX) {
        drive.driveRobotCentric(
                leftX,
                leftY,
                rightX
        );
    }

    public void updateRobotPoseMT1() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");

                Pose2d cameraPose = new Pose2d(
                        botpose.getPosition().x,
                        botpose.getPosition().y,
                        Rotation2d.fromDegrees(botpose.getOrientation().getYaw(AngleUnit.DEGREES)));
                odometry.updatePose(cameraPose);
            }
        }
    }

    public void updateRobotPoseMT2() {
        LLResult result = limelight.getLatestResult();

        limelight.updateRobotOrientation(odometry.getPose().getHeading());
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

                Pose2d cameraPose = new Pose2d(
                        botpose.getPosition().x,
                        botpose.getPosition().y,
                        Rotation2d.fromDegrees(botpose.getOrientation().getYaw(AngleUnit.DEGREES)));
                odometry.updatePose(cameraPose);
            }
        }
    }

    public void setOdometry (Pose2d pose) {
        odometry.updatePose(pose);
    }




}
