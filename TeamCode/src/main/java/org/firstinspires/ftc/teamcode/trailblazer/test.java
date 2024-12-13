package org.firstinspires.ftc.teamcode.trailblazer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.firstinspires.ftc.teamcode.trailblazer.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.trailblazer.path.Path;
import org.firstinspires.ftc.teamcode.trailblazer.path.PathBuilder;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Vector2D;

import org.fotmrobotics.trailblazer.MathKt;

@TeleOp
public class test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        PIDF headingPID = new PIDF(0.05, 0, 0, 0);

        boolean rotate = false;
        double targetDriveHeading = 0;

        waitForStart();

        if (isStopRequested()) {return;}

        while (opModeIsActive()) {
            drive.mecanumDrive(gamepad1);
            //drive.trueNorthDrive(gamepad1);
            //Pose2D driveVector = path1.test();
            //telemetry.addData("X", driveVector.getX());
            Pose2D currentPos = drive.odometry.getPosition();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            telemetry.addData("r?", r);

            double currentHeading = drive.odometry.getPosition().getH();

            if (r != 0) {
                targetDriveHeading = currentHeading + r * -90;
                targetDriveHeading = MathKt.angleWrap(targetDriveHeading);
                //targetDriveHeading = targetDriveHeading > 180 ? targetDriveHeading - 360 : targetDriveHeading < -180 ? targetDriveHeading - 360 : targetDriveHeading;
                rotate = true;
            } else if (rotate) {
                targetDriveHeading = currentHeading;
                rotate = false;
            }

            telemetry.addData("targetDriveHeading", targetDriveHeading);
            telemetry.addData("currentHeading", currentHeading);

            Pose2D pose2D = new Pose2D(x, y, targetDriveHeading);

            telemetry.addData("targetX", pose2D.getX());
            telemetry.addData("targetY", pose2D.getY());
            telemetry.addData("targetH", pose2D.getH());

            double angle = Math.toDegrees(Math.atan2(pose2D.getY(), pose2D.getX())) + 90;
            angle = angle <= 180 ? angle : angle - 360;
            //angle -= pose2D.getH();

            double headingError = pose2D.getH() - currentPos.getH();
            headingError = MathKt.angleWrap(headingError);
            headingError = headingError > 180 ? headingError - 360 : headingError < -180 ? headingError - 360 : headingError;

            telemetry.addData("headingError", headingError);

            double headingOut = headingPID.update(headingError);

            telemetry.addData("headingOut", headingOut);

            double power = Math.min(pose2D.norm(), 1);

            double x1 = Math.sin(Math.toRadians(angle)) * power;
            double y1 = -Math.cos(Math.toRadians(angle)) * power;
            double r1 = Math.min(Math.abs(headingOut),1) * (Math.abs(headingError) / -headingError); // * (1 + (Boolean.compare(headingError > 0, false) * -2))
            //double r1 = (1 + (Boolean.compare(headingError > 0, false) * -2)) * Math.min(Math.abs(headingOut),1);

            telemetry.addData("x:", x1);
            telemetry.addData("y:", y1);
            telemetry.addData("r:", r1);
            telemetry.update();
        }
    }
}
