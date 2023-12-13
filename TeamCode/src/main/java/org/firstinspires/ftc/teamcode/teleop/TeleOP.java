package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name="TeleOP", group="TeleOP")
public class TeleOP extends LinearOpMode {

    double claw1Grab = 0.667;
    double claw2Grab = 0.667;
    Gamepad currentGamepad1, previousGamepad1;
    Gamepad currentGamepad2, previousGamepad2;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        waitForStart();
        double straight, turn, strafe;
        double intakePower,liftPower;
            while (opModeIsActive()) {
                currentGamepad1 = gamepad1;
                previousGamepad1 = currentGamepad1;
                currentGamepad2 = gamepad2;
                previousGamepad2 = currentGamepad2;

                straight = (-gamepad1.left_stick_y > 0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.3) : (-gamepad1.left_stick_y < -0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.3) : 0;
                strafe = (gamepad1.left_stick_x > 0.05) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.3) : (gamepad1.left_stick_x < -0.05) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.3) : 0;
                turn = (gamepad1.right_stick_x > 0.05) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.3) : (gamepad1.right_stick_x < -0.05) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.3) : 0;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight*1, -strafe*1), -turn*0.3));
                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

               

                intakePower = (gamepad1.left_bumper)?1:gamepad1.right_trigger>0?-1:0;
                liftPower = (gamepad1.right_bumper)?1:-gamepad1.right_trigger;




                drive.claw1.setPosition(gamepad1.left_trigger);
                drive.claw2.setPosition(gamepad1.right_trigger);

                telemetry.addData("claw1",gamepad1.left_trigger);
                telemetry.addData("claw2",gamepad1.right_trigger);

                telemetry.update();
            }

    }
}
