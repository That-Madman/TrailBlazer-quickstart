package org.firstinspires.ftc.teamcode.trailblazer.drivebase;

import static org.fotmrobotics.trailblazer.MathKt.angleWrap;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.trailblazer.path.PathBuilder;
import org.fotmrobotics.trailblazer.PIDF;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Controls the drive base. Currently supports mecanum drive base.
 *
 * @author Preston Cokis
 */
public class Drive {
    HardwareMap hardwareMap;
    public Odometry odometry;

    DriveValues driveValues = new DriveValues();

    public double xScale = driveValues.xScale;
    public double yScale = driveValues.yScale;
    public double angularScale = driveValues.angularScale;

    private final PIDF positionPID = driveValues.positionPID;
    private final PIDF headingPID = driveValues.headingPID;

    public Drive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        odometry = new Odometry(hardwareMap);

        setMotors(driveValues.motorNames);

        for (int i : driveValues.reverseMotors) {
            motors.get(i).setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    private final List<DcMotor> motors = new ArrayList<>();
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
                (y * yScale + x * xScale + r * angularScale),
                (y * yScale - x * xScale - r * angularScale),
                (y * yScale - x * xScale + r * angularScale),
                (y * yScale + x * xScale - r * angularScale)
        };
    }

    public void moveVector(Vector2D v, double r, boolean relative) {
        Pose2D currentPos = odometry.getPosition();

        double angle = angleWrap(Math.toDegrees(Math.atan2(v.getY(), v.getX())) + 180);

        if (relative) {angle -= currentPos.getH();}

        double power = Math.min(v.norm(), 1);

        double x = Math.cos(Math.toRadians(angle)) * power;
        double y = Math.sin(Math.toRadians(angle)) * power;

        runMotors(getPowers(x, y, r));
    }

    public void moveVector(Pose2D pose, boolean relative) {
        Pose2D currentPos = odometry.getPosition();

        double headingError = pose.getH() - currentPos.getH();
        headingError = angleWrap(headingError);
        headingError = (headingError > 180) ? headingError - 360 :
                (headingError < -180) ? headingError - 360 : headingError;

        double headingOut = headingPID.update(headingError);

        double r = Math.min(Math.abs(headingOut), 1) * (Math.abs(headingError) / headingError);

        moveVector(new Vector2D(pose.getX(), pose.getY()), r, relative);
    }

    public void moveVector(Pose2D pose) {
        moveVector(pose, true);
    }

    public Pose2D target;
    public void movePoint(Pose2D point) {
        target = point;
        Pose2D currentPos = odometry.getPosition();

        Vector2D difference = point.minus(currentPos);

        double out = Math.min(positionPID.update(difference.norm()), 1);

        Vector2D result = difference.times(out);

        moveVector(new Pose2D(result.getX(), result.getY(), point.getH()), true);
    }

    public void movePoint(Vector2D point, double angle) {
        movePoint(new Pose2D(point.getX(), point.getY(), angle));
    }

    private int timesChecked = 0;
    private Pose2D lastPos = new Pose2D(Double.NaN, Double.NaN, Double.NaN);
    public boolean atTarget() {
        Pose2D currentPos = odometry.getPosition();

        timesChecked = Math.sqrt(
                Math.pow(currentPos.getX() - lastPos.getX(), 2) +
                Math.pow(currentPos.getY() - lastPos.getY(), 2)) < 0.05 &&
                Math.abs(currentPos.getH() - lastPos.getH()) < 0.05 ?
                timesChecked + 1 : 0;
        lastPos.set(currentPos);
        return timesChecked > 2;
    }

    private boolean rotate = true;
    private double targetDriveHeading = 0;
    public void mecanumDrive (Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        double currentHeading = odometry.getPosition().getH();

        if (r != 0) {
            moveVector(new Vector2D(x, y), -r, false);
            rotate = true;
        } else if (rotate) {
            targetDriveHeading = currentHeading;
            rotate = false;
        }

        if (!rotate) {moveVector(new Pose2D(x, y, targetDriveHeading), false);}
    }

    public void trueNorthDrive(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double r = gamepad.right_stick_x;

        if (gamepad.right_stick_button) {odometry.resetHeading();}

        double currentHeading = odometry.getPosition().getH();

        if (r != 0) {
            moveVector(new Vector2D(x, y), -r, true);
            rotate = true;
        } else if (rotate) {
            targetDriveHeading = currentHeading;
            rotate = false;
        }

        if (!rotate) {moveVector(new Pose2D(x, y, targetDriveHeading));}
    }

    public PathBuilder PathBuilder(Vector2D startPoint) {
        return new PathBuilder(this, startPoint);
    }
}
