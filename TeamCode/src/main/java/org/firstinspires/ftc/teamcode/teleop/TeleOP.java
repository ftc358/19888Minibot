package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name="TeleOP", group="TeleOP")
public class TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            double straight, turn, strafe;
            double liftPower;
            while (opModeIsActive()) {
                straight = (-gamepad1.left_stick_y > 0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.3) : (-gamepad1.left_stick_y < -0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.3) : 0;
                strafe = (gamepad1.left_stick_x > 0.05) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.3) : (gamepad1.left_stick_x < -0.05) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.3) : 0;
                turn = (gamepad1.right_stick_x > 0.05) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.3) : (gamepad1.right_stick_x < -0.05) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.3) : 0;



                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                straight*0.4,
                                -strafe*0.4
                        ),
                        -turn*0.3
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
















                telemetry.update();
            }

    }
}
