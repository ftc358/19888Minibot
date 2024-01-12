package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
        double liftPower, claw1Pos, claw2Pos, wristPos, armPos;
        int transferState = 0;
        int dropState = 0;

        while (opModeIsActive()) {
            currentGamepad1 = gamepad1;
            previousGamepad1 = currentGamepad1;

            straight = (-currentGamepad1.left_stick_y > 0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) + 0.3) : (-gamepad1.left_stick_y < -0.05) ? (Math.pow(-gamepad1.left_stick_y, 3) - 0.3) : 0;
            strafe = (gamepad1.left_stick_x > 0.05) ? (Math.pow(gamepad1.left_stick_x, 3) + 0.3) : (gamepad1.left_stick_x < -0.05) ? (Math.pow(gamepad1.left_stick_x, 3) - 0.3) : 0;
            turn = (gamepad1.right_stick_x > 0.05) ? (Math.pow(gamepad1.right_stick_x, 5) + 0.3) : (gamepad1.right_stick_x < -0.05) ? (Math.pow(gamepad1.right_stick_x, 5) - 0.3) : 0;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(straight * 1, -strafe * 1), -turn * 0.3));
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

            //IMU Feedback data
            YawPitchRollAngles angles = drive.imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch",angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll",angles.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw",angles.getYaw(AngleUnit.DEGREES));


            if (currentGamepad1.cross && !previousGamepad1.cross){
                transferState = (transferState +1)%4;
            }
            if ((currentGamepad1.circle && !previousGamepad1.circle) && transferState >=2){
                transferState = 4;
            }

//            switch (transferState){
//                case 0:
//                    claw1Pos = 0;
//                    claw2Pos = 0;
//                    wristPos = 0;
//                    armPos = 0;
//                    break;
//                case 1://grabs two
//                    claw1Pos = 0.51;
//                    claw2Pos = 0.28;
//                    break;
//                case 2://place front
//                    wristPos = 0.2;
//                    armPos = 0.3;
//                    break;
//                case 3://place 3
//                    wristPos = 1.0;
//                    aPos = 0.7;
//                    break;
//
//                case 6:
//            }
            claw1Pos = gamepad2.left_stick_x;
            claw2Pos = gamepad2.right_stick_x;
            //0.28
            //0.51

            armPos = currentGamepad1.left_trigger;
            //0.7 all the way back, 0.05 off the ground a bit, assuming 0.3 for front placement.
            wristPos = currentGamepad1.right_trigger;
            //1.0 max

            if (gamepad1.)

//            if (gamepad1.a) {
//
//            }
//            else if (gamepad1.b){
//                armPos = 0;
//                wristPos = 0;
//            }else {
//                armPos = 0.7;
//                wristPos = 1;
//            }
            telemetry.addData("armPos",armPos);
            telemetry.addData("wristPos",wristPos);
            telemetry.addData("claw1Pos",claw1Pos);
            telemetry.addData("claw2Pos",claw2Pos);




            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.wrist.setPosition(wristPos);
            drive.arm1.setPosition(armPos);
            drive.arm2.setPosition(armPos);
            telemetry.update();
        }

    }
}
