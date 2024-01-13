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
    double wrist2Pos = 1.0;
    double arm1Pos = 0.3;
    double arm2Pos = 0.7;
    //-------------------------------------------

    //other stuff idk
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.imu.resetYaw();

        waitForStart();
        double straight, turn, strafe;

        double liftPower, claw1Pos, claw2Pos, wristPos, armPos;
        armPos = 0;
        wristPos = 0;
        claw1Pos = 0;
        claw2Pos = 0;

        int transferState = 0;
        int intakeState = 0;

        //transfer bools
        boolean intake1 = false;

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);


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
            telemetry.addData("Yaw",angles.getYaw(AngleUnit.DEGREES));
            //------------------------------------------------------------------------------


            //INTAKE LMAOOOOOOO ------------------------------------------
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
               intakeState = (intakeState +1)%5;
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                transferState = (transferState + 1)%3;
            }

            switch (intakeState){
                case 0:
                    claw1Pos= 0;
                    claw2Pos = 0;
                    break;
                case 1:
                    claw1Pos = claw1Grab;
                    claw2Pos = claw2Grab;
                    break;
                case 2:
                    claw2Pos = 0;
                    break;
                case 3:
                    claw1Pos = 0;
                    break;
                case 4:
                    armPos = 0;
                    wristPos = 0;
                    transferState = 0;
                    intakeState = 0;
                    break;
            }

            if (gamepad1.dpad_left){drive.imu.resetYaw();}

            //TRANSFER TSTUFFFFFFFFF ----------------------------------------

            switch (transferState){
                case 0:
                    armPos = 0;
                    wristPos = 0;
                    break;
                case 1:
                    armPos = arm1Pos;
                    wristPos = wrist1Pos;
                    break;
                case 2:
                    armPos = arm2Pos;
                    wristPos = wrist2Pos;
                    break;
            }


            //------------------------------------------------------------------


            //LIFT LIFT ------------------------------------------------------------
            if (gamepad1.left_trigger >= 0.1){
                //lift go up
                liftPower = gamepad1.left_trigger/2;
            }
            else if (gamepad1.left_bumper){
                //lift go down
                liftPower = -0.2;            }
            else{
                liftPower = 0;
            }
            //------------------------------------------------------------------
            telemetry.addData("TransState",transferState);
            telemetry.addData("INTState",intakeState);
            telemetry.addData("armPos",armPos);
            telemetry.addData("wristPos",wristPos);
            telemetry.addData("claw1Pos",claw1Pos);
            telemetry.addData("claw2Pos",claw2Pos);

            drive.claw1.setPosition(claw1Pos);
            drive.claw2.setPosition(claw2Pos);
            drive.wrist.setPosition(wristPos);
            drive.arm1.setPosition(armPos);
            drive.arm2.setPosition(armPos);
            drive.lift1.setPower(liftPower);
            drive.lift2.setPower(liftPower);
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
        }
    }
}