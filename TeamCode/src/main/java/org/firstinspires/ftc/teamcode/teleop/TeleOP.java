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

    //positions to tune -------------------------
    double claw1Grab = 0.51;
    double claw2Grab = 0.28;
    double wrist1Pos = 0.2;
    double wrist2Pos = 0.4;
    double arm1Pos = 0.3;
    double arm2Pos = 0.7;
    //-------------------------------------------

    //other stuff idk
    Gamepad currentGamepad1, previousGamepad1;
    Gamepad currentGamepad2, previousGamepad2;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        waitForStart();
        double straight, turn, strafe;

        double liftPower, claw1Pos, claw2Pos, wristPos, armPos;
        int transferState = 0;
        int intakeState = 0;

        //transfer bools
        boolean intake1 = false;

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

            //IMU Feedback data------------------------------------------------------------
            YawPitchRollAngles angles = drive.imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch",angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll",angles.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw",angles.getYaw(AngleUnit.DEGREES));
            //------------------------------------------------------------------------------


            //INTAKE LMAOOOOOOO ------------------------------------------
            if (gamepad1.right_bumper && intakeState == 0){
                claw1Pos = claw1Grab;
                intakeState = 1;
            }
            else if (gamepad1.right_bumper && intakeState == 1){
                claw2Pos = claw2Grab;
                intakeState = 2;
            }
            else if (gamepad1.right_bumper && intakeState == 2){
                claw2Pos = 0;
                intakeState = 3;
            }
            else if (gamepad1.right_bumper && intakeState == 3){
                claw1Pos = 0;
                armPos = 0;
                wristPos = 0;
                intakeState = 0;
                transferState = 0;
            }
            //------------------------------------------------------------

            //TRANSFER TSTUFFFFFFFFF ------------------------------------------
            if (gamepad1.x && transferState == 0){
                //drop front
                armPos = arm1Pos;
                wristPos = wrist1Pos;
                transferState = 1;
            }
            else if (gamepad1.x && transferState == 1){
                //drop back
                armPos = arm2Pos;
                wristPos = wrist2Pos;
                transferState = 2;
            }
            else if (gamepad1.x && transferState == 2){
                //reset
                armPos = 0;
                wristPos = 0;
                transferState = 0;
            }
            //------------------------------------------------------------------


            //LIFT LIFT ------------------------------------------------------------
            else if (gamepad1.left_trigger >= 0.1){
                //lift go up
                drive.lift1.setPower(gamepad1.left_trigger);
                drive.lift2.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.left_bumper){
                //lift go down
                drive.lift1.setPower(-0.4);
                drive.lift2.setPower(0.4);
            }
            else{
                //lift hold
                drive.lift1.setPower(0.05);
                drive.lift2.setPower(-0.05);
            }
            //------------------------------------------------------------------


            //=---------------------------------------------
            claw1Pos = gamepad2.left_stick_x;
            claw2Pos = gamepad2.right_stick_x;
            //0.28
            //0.51

            armPos = currentGamepad1.left_trigger;
            //0.7 all the way back, 0.05 off the ground a bit, assuming 0.3 for front placement.
            wristPos = currentGamepad1.right_trigger;
            //1.0 max
            //------------------------------------------------


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
