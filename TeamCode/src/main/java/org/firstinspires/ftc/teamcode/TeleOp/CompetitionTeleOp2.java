package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Shared.Drive;


@TeleOp(name="Competition 2 OBJ", group="TeleOp")
//@Disabled
public class CompetitionTeleOp2 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    private Servo Load;
    private DcMotorEx Ring;
    private DcMotor Elevate, Sweep;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LM DT");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RM DT");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LR DT");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RR DT");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        Load = hardwareMap.get(Servo.class, "Load");
        Ring = hardwareMap.get(DcMotorEx.class, "Ring");
        Elevate = hardwareMap.get(DcMotor.class, "Elevate");
        Sweep = hardwareMap.get(DcMotor.class, "Sweep");

        Ring.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Drive drive = new Drive(this);
        drive.init();

        waitForStart();

        Load.setPosition(0);

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
            double magRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double thetaRight = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double magLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double thetaLeft = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double pi = Math.PI;

        /*if (thetaRight > 0 && thetaRight < pi / 2) {
            frontRightPowerFactor = -Math.cos(2 * thetaRight);
        } else if (thetaRight >= -pi && thetaRight < -pi / 2) {
            frontRightPowerFactor = Math.cos(2 * thetaRight);
        } else if (thetaRight >= pi / 2 && thetaRight <= pi) {
            frontRightPowerFactor = 1;
        } else {
            frontRightPowerFactor = -1;
        }

        if (thetaLeft > 0 && thetaLeft < pi / 2) {
            backLeftPowerFactor = -Math.cos(2 * thetaLeft);
        } else if (thetaLeft >= -pi && thetaLeft < -pi / 2) {
            backLeftPowerFactor = Math.cos(2 * thetaLeft);
        } else if (thetaLeft >= pi / 2 && thetaLeft <= pi) {
            backLeftPowerFactor = 1;
        } else {
            backLeftPowerFactor = -1;
        }

        if (thetaRight > -pi / 2 && thetaRight < 0) {
            backRightPowerFactor = Math.cos(2 * thetaRight);
        } else if (thetaRight > pi / 2 && thetaRight < pi) {
            backRightPowerFactor = -Math.cos(2 * thetaRight);
        } else if (thetaRight >= 0 && thetaRight <= pi / 2) {
            backRightPowerFactor = 1;
        } else {
            backRightPowerFactor = -1;
        }

        if (thetaLeft > -pi / 2 && thetaLeft < 0) {
            frontLeftPowerFactor = Math.cos(2 * thetaLeft);
        } else if (thetaLeft > pi / 2 && thetaLeft < pi) {
            frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
        } else if (thetaLeft >= 0 && thetaLeft <= pi / 2) {
            frontLeftPowerFactor = 1;
        } else {
            frontLeftPowerFactor = -1;
        }*/

            if (thetaRight > 0 && thetaRight < pi / 2) {
                frontRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= -pi && thetaRight < -pi / 2) {
                frontRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight >= pi / 2 && thetaRight <= pi) {
                frontRightPowerFactor = 1;
            } else {
                frontRightPowerFactor = -1;
            }

            if (thetaLeft > 0 && thetaLeft < pi / 2) {
                backLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= -pi && thetaLeft < -pi / 2) {
                backLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= pi / 2 && thetaLeft <= pi) {
                backLeftPowerFactor = 1;
            } else {
                backLeftPowerFactor = -1;
            }

            if (thetaRight > -pi / 2 && thetaRight < 0) {
                backRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight > pi / 2 && thetaRight < pi) {
                backRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= 0 && thetaRight <= pi / 2) {
                backRightPowerFactor = 1;
            } else {
                backRightPowerFactor = -1;
            }

            if (thetaLeft > -pi / 2 && thetaLeft < 0) {
                frontLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft > pi / 2 && thetaLeft < pi) {
                frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= 0 && thetaLeft <= pi / 2) {
                frontLeftPowerFactor = 1;
            } else {
                frontLeftPowerFactor = -1;
            }

            // for test robot (just chassis & camera)
//        if(gamepad1.a){
//            leftFrontDrive.setPower(0.5);
//            leftBackDrive.setPower(0.5);
//            rightFrontDrive.setPower(0.5);
//            rightBackDrive.setPower(0.5);
//        }else if(gamepad1.b){
//            leftFrontDrive.setPower(-0.5);
//            leftBackDrive.setPower(-0.5);
//            rightFrontDrive.setPower(-0.5);
//            rightBackDrive.setPower(-0.5);
//        }else{
//            leftFrontDrive.setPower(frontLeftPowerFactor * magLeft);
//            rightFrontDrive.setPower(frontRightPowerFactor * magRight);
//            leftBackDrive.setPower(backLeftPowerFactor * magLeft);
//            rightBackDrive.setPower(backRightPowerFactor * magRight);
//        }

            // for actual competition robot
//        if(gamepad1.a){
//            leftFrontDrive.setPower(-0.5);
//            leftBackDrive.setPower(-0.5);
//            rightFrontDrive.setPower(-0.5);
//            rightBackDrive.setPower(-0.5);
//        }else if(gamepad1.b){
//            leftFrontDrive.setPower(0.5);
//            leftBackDrive.setPower(0.5);
//            rightFrontDrive.setPower(0.5);
//            rightBackDrive.setPower(0.5);
//        }else{
            leftFrontDrive.setPower(frontLeftPowerFactor * magLeft);
            rightFrontDrive.setPower(-frontRightPowerFactor * magRight);
            leftBackDrive.setPower(backLeftPowerFactor * magLeft);
            rightBackDrive.setPower(-backRightPowerFactor * magRight);
//        }

            double launcherSpeed;
            if (gamepad2.right_trigger > 0.5) {
                launcherSpeed = 280;
            } else if (gamepad2.left_trigger > 0.5) {
                launcherSpeed = 260;
            } else {
                launcherSpeed = 0;
            }

            Ring.setVelocity(launcherSpeed, AngleUnit.DEGREES);

            Elevate.setPower(0.5 * gamepad2.right_stick_y);

            if (gamepad2.x) {
                Sweep.setPower(-1);
            }
            if (gamepad2.y) {
                Sweep.setPower(0);
            }
            if (gamepad2.a) {
                Load.setPosition(0.67);
                sleep(900);
                Load.setPosition(0);
            }
            if (gamepad2.b) {
                Sweep.setPower(1);
            }

            telemetry.addData("IMU Orientation: ", drive.getImuDeltaAngle());

            telemetry.addData("front right power ", ((float) Math.round(rightFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("front left power ", ((float) Math.round(leftFrontDrive.getPower() * 100)) / 100);
            telemetry.addData("back right power ", ((float) Math.round(rightBackDrive.getPower() * 100)) / 100);
            telemetry.addData("back left power ", ((float) Math.round(leftBackDrive.getPower() * 100)) / 100);
            telemetry.addData("left joystick x", ((float) Math.round(gamepad1.left_stick_x * 100)) / 100);
            telemetry.addData("left joystick y", ((float) Math.round(-gamepad1.left_stick_y * 100)) / 100);
            telemetry.addData("magnitude left", ((float) Math.round(magLeft * 100)) / 100);
            telemetry.addData("thetaLeft", ((float) Math.round(thetaLeft / pi * 100)) / 100);

            telemetry.update();
        }
    }
}
