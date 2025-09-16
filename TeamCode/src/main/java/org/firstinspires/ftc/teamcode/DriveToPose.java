package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import java.util.function.Supplier;

public class DriveToPose extends CommandBase {

    private  DriveSubsystem drive;
    private Supplier<Pose2d> currentPose = drive::getCurrentPose;
    private Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    private ProfiledPIDController translationPID = new ProfiledPIDController(
            0.001, 0, 0,
            new TrapezoidProfile.Constraints(3, 3)
    );
    private ProfiledPIDController rotationPID = new ProfiledPIDController(
            0.001, 0, 0,
            new TrapezoidProfile.Constraints(3, 3)
    );


    public DriveToPose(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double distanceAwayX = currentPose.get().getX() - targetPose.getX();
        double distanceAwayY = currentPose.get().getY() - targetPose.getY();
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        double translationOutput = translationPID.calculate(distanceAway, 0);


        double newRotation = rotationPID.calculate(currentPose.get().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        double newForward = translationOutput * Math.sin(angleOfDistance);
        double newStrafe = translationOutput * Math.cos(angleOfDistance);

        drive.fieldCentricDrive(newForward, newStrafe, newRotation);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
