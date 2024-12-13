package org.firstinspires.ftc.teamcode.trailblazer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trailblazer.drivebase.Drive;
import org.firstinspires.ftc.teamcode.trailblazer.path.Path;
import org.firstinspires.ftc.teamcode.trailblazer.path.PathBuilder;
import org.fotmrobotics.trailblazer.Pose2D;
import org.fotmrobotics.trailblazer.Vector2D;

@TeleOp
public class autoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);

        Path path1 = new PathBuilder(drive, new Vector2D(0,0))
                .pt(new Vector2D(0, 24))
                .pt(new Vector2D(24,24))
                .pt(new Vector2D(24, 0))
                .pt(new Vector2D(24, 0))
                .build();

        waitForStart();

        if (isStopRequested()) {return;}

        while (opModeIsActive()) {
            if (gamepad1.a) {
                path1.run();
            }
        }
    }
}
