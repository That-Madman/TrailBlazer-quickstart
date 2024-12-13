package org.firstinspires.ftc.teamcode.trailblazer;


import static org.fotmrobotics.trailblazer.PathKt.driveVector;
import static org.fotmrobotics.trailblazer.SplineKt.curvature;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.MecanumDrive;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.PathKt;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Spline;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;


@TeleOp
public class NewSplineTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        ArrayList controlPoints = new ArrayList();
        controlPoints.add(new Vector2D(0,0));
        controlPoints.add(new Vector2D(0,0));
        controlPoints.add(new Vector2D(0,48));
        controlPoints.add(new Vector2D(12,48));
        controlPoints.add(new Vector2D(12,0));
        controlPoints.add(new Vector2D(12,0));
        Spline spline = new Spline(controlPoints);

        PIDF pidf = new PIDF (0.5,0, 0, 0);

        spline.setSegment(0);

        waitForStart();

        if (isStopRequested()) {return;}

        while (opModeIsActive()) {
            Pose2D pos = drive.getPosition();
            Pose2D newPos = pos;

            //ArrayList<Vector2D> testpoints = spline.getSegment();
            double t = spline.getClosestPoint(newPos);
            //Double test = spline.getClosest(newPos);
            Vector2D splinePoint = spline.getPoint(t);
            Vector2D splineDeriv = spline.getDeriv(t);
            Vector2D splineDeriv2 = spline.getDeriv2(t);

            telemetry.addLine("Current");
            telemetry.addData("X", newPos.getX());
            telemetry.addData("Y", newPos.getY());
            telemetry.addData("H", newPos.getH());

            telemetry.addLine("Test");
            telemetry.addData("T Value", t);
            //telemetry.addData("X", testpoints.get(3).getX());
            //telemetry.addData("Y", testpoints.get(3).getY());

            telemetry.addLine("Closest Spline");
            telemetry.addData("X", splinePoint.getX());
            telemetry.addData("Y", splinePoint.getY());
            telemetry.addData("t", t);

            telemetry.addLine("Derivative");
            telemetry.addData("X", splineDeriv.getX());
            telemetry.addData("Y", splineDeriv.getY());

            telemetry.update();

            Pose2D driveVector = driveVector(spline, newPos, pidf);
            telemetry.addLine("Drive Vector");
            telemetry.addData("X", driveVector.getX());
            telemetry.addData("Y", driveVector.getY());
            telemetry.addData("H", driveVector.getH());

            double tTest = spline.getClosestPoint(pos);

            Vector2D splinePointTest = spline.getPoint(t);
            Vector2D splineDerivTest = spline.getDeriv(t);

            double angle = Math.atan2(splineDeriv.getY(), splineDeriv.getX());
            //if (splineDeriv.x < 0) angle += Math.PI
            //var angle = atan2(splineDeriv.y, splineDeriv.x)
            //val splineDeriv2 = spline.getDeriv2(t)

            Vector2D forward = splineDeriv;

            double k = curvature(spline, tTest, 1e-4);
            double centripetalMagnitude = Math.pow(forward.norm(),2.0) * k;
            Vector2D centripetal = new Vector2D(
                    centripetalMagnitude * Math.cos(angle + Math.PI / 2),
                    centripetalMagnitude * Math.sin(angle + Math.PI / 2)
            );

            telemetry.addLine("Centripetal Vector");
            telemetry.addData("X", centripetal.getX());
            telemetry.addData("Y", centripetal.getY());
            telemetry.addData("Magnitude", centripetalMagnitude);
            telemetry.addData("Angle", angle + Math.PI / 2);

            double[] powers = drive.powersToTarget(new Pose2D(pos.getX() + driveVector.getX(), pos.getY() + driveVector.getY(), 0));
            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("frontRight", powers[1]);
            telemetry.addData("backLeft", powers[2]);
            telemetry.addData("backRight", powers[3]);

            if (gamepad1.a) {
                drive.toTarget(new Pose2D(pos.getX() + driveVector.getX(), pos.getY() + driveVector.getY(), 0));
                if (t >= 1) {
                    spline.incSegment();
                }
            } else {
                drive.trueNorthDrive(gamepad1);
            }
        }
    }
}
