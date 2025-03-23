package org.firstinspires.ftc.teamcode.trailblazer.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Odometry;
import org.fotmrobotics.trailblazer.Pose2D;

@TeleOp
public class OTOS_tuning extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Odometry otos = new Odometry(hardwareMap);

        waitForStart();

        if (isStopRequested()) {return;}

        while (opModeIsActive()) {
            Pose2D pos = otos.getPosition();
            telemetry.addData("X", pos.getX());
            telemetry.addData("Y", pos.getY());
            telemetry.addData("H", pos.getH());
            telemetry.update();
        }
    }
}
