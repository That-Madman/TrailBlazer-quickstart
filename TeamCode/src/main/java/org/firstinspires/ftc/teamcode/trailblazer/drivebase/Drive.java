package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import static org.fotmrobotics.trailblazer.MathKt.angleWrap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trailblazer.path.PathBuilder;
import org.fotmrobotics.trailblazer.MathKt;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class Drive {
    HardwareMap hardwareMap;
    public Odometry odometry;

    /**
     * 0. Front Left
     * 1. Front Right
     * 2. Back Left
     * 3. Back Right
     */
    String[] motorNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
    };

    private PIDF positionPID = new PIDF(0.125, 0, 0, 0);
    private PIDF headingPID = new PIDF(0.01, 0, 0, 0);

    //double maxSpeed = 50;

    public Drive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        odometry = new Odometry(hardwareMap);

        setMotors(motorNames);

        // TODO: Reverse any motors if necessary.
        motors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(2).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private List<DcMotor> motors = new ArrayList();
    public void setMotors(String[] motorNames) {
        for (int i = 0; i <= 3; i++) {
            motors.add(this.hardwareMap.get(DcMotor.class, motorNames[i]));
        }
    }

    public void runMotors(double[] powers) {
        for (int i = 0; i <= 3; i++) {
            this.motors.get(i).setPower(powers[i]);
        }
    }

    public double[] getPowers(double x, double y, double r) {
        return new double[] {
                (y + x + r) * 0.8,
                (y - x - r) * 0.8,
                (y - x + r) * 0.8,
                (y + x - r) * 0.8
        };
    }

    public void moveVector(Pose2D pose2D) {
        Pose2D currentPos = odometry.getPosition();

        double angle = Math.toDegrees(Math.atan2(pose2D.getY(), pose2D.getX())) + 90; // Adds 90 bc of how angles are read by sparkfun, maybe change
        angle = angle <= 180 ? angle : angle - 360;
        angle -= currentPos.getH();

        double headingError = pose2D.getH() - currentPos.getH();
        headingError = angleWrap(headingError);
        headingError = headingError > 180 ? headingError - 360 : headingError < -180 ? headingError - 360 : headingError;

        double headingOut = headingPID.update(headingError);

        double power = Math.min(pose2D.norm(), 1);

        double x = Math.sin(Math.toRadians(angle)) * power;
        double y = -Math.cos(Math.toRadians(angle)) * power;
        double r = Math.min(Math.abs(headingOut),1) * (Math.abs(headingError) / -headingError); // * (1 + (Boolean.compare(headingError > 0, false) * -2))

        runMotors(getPowers(x, y, r));
    }

    public void moveVector(Vector2D v, double angle) {
        moveVector(new Pose2D(v.getX(), v.getY(), angle));
    }

    public void moveVector(Vector2D v) {
        double angle = odometry.getPosition().getH();
        moveVector(v, angle);
    }

    public void movePoint(Pose2D point) {
        Pose2D currentPos = odometry.getPosition();
        Vector2D difference = point.minus(currentPos);
        moveVector(difference, point.getH());
    }

    public void movePoint(Vector2D point, double angle) {
        movePoint(new Pose2D(point.getX(), point.getY(), angle));
    }

    private boolean rotate = false;
    private double targetDriveHeading = 0;
    public void mecanumDrive (Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        /// Kaden's control scheme (except not right, whoops)
        /// double x = gamepad.left_stick_x;
        /// double y = -gamepad.right_stick_y;
        /// double r = gamepad.right_stick_x;

        double currentHeading = odometry.getPosition().getH();

        if (r != 0) {
            targetDriveHeading = currentHeading + r * -90;
            targetDriveHeading = angleWrap(targetDriveHeading);
            rotate = true;
        } else if (rotate) {
            targetDriveHeading = currentHeading;
            rotate = false;
        }

        moveVector(new Pose2D(x, y, targetDriveHeading));
    }

    public void trueNorthDrive(Gamepad gamepad) {
        odometry.update();

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        double angle = Math.toDegrees(Math.atan2(y, x)) + 90;
        angle = (angle <= 180 ? angle : angle - 360) - odometry.currentPos.getH();
        x = Math.sin(Math.toRadians(angle)) * power;
        y = (-Math.cos(Math.toRadians(angle))) * power;

        runMotors(new double[] {
                y + x + r,
                y - x - r,
                y - x + r,
                y + x - r
        });
    }

    public PathBuilder pathBuilder(Vector2D startPoint) {
        return new PathBuilder(this, startPoint);
    }
}
