package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class Drive {
    public DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
    LinearOpMode opMode;
    Telemetry telemetry;
    public HardwareMap hardwareMap; // will be set in OpModeManager.runActiveOpMode
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;

    HashMap <DcMotor, Integer> motorInitialPositions, motorTargetPositions;
    HashMap <DcMotor, Double> motorPowerFactors;


    public Drive(LinearOpMode _opMode){
        opMode = _opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        motorInitialPositions = new HashMap<>();
        motorTargetPositions = new HashMap<>();
        motorPowerFactors = new HashMap<>();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = opMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = opMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = opMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void vroom_vroom (double magRight, double thetaRight, double magLeft, double thetaLeft, double timeout) {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double rightFrontPowerFactor, leftFrontPowerFactor, rightBackPowerFactor, leftBackPowerFactor;
        double pi = Math.PI;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path",  "Starting at %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // reset the timeout time and start motion.
        runtime.reset();

        if(thetaRight > 0 && thetaRight < pi/2){
            rightFrontPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= -pi && thetaRight < -pi/2){
            rightFrontPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight >= pi/2 && thetaRight <= pi){
            rightFrontPowerFactor = 1;
        }else{
            rightFrontPowerFactor = -1;
        }

        if(thetaLeft > 0 && thetaLeft < pi/2) {
            leftBackPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= -pi && thetaLeft < -pi/2){
            leftBackPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= pi/2 && thetaLeft <= pi){
            leftBackPowerFactor = 1;
        }else{
            leftBackPowerFactor = -1;
        }

        if(thetaRight > -pi/2 && thetaRight < 0) {
            rightBackPowerFactor = Math.cos(2 * thetaRight);
        }else if(thetaRight > pi/2 && thetaRight < pi){
            rightBackPowerFactor = -Math.cos(2 * thetaRight);
        }else if(thetaRight >= 0 && thetaRight <= pi/2){
            rightBackPowerFactor = 1;
        }else{
            rightBackPowerFactor = -1;
        }

        if(thetaLeft > -pi/2 && thetaLeft < 0) {
            leftFrontPowerFactor = Math.cos(2 * thetaLeft);
        }else if(thetaLeft > pi/2 && thetaLeft < pi){
            leftFrontPowerFactor = -Math.cos(2 * thetaLeft);
        }else if(thetaLeft >= 0 && thetaLeft <= pi/2){
            leftFrontPowerFactor = 1;
        }else{
            leftFrontPowerFactor = -1;
        }

        motorPowerFactors.put(leftFrontDrive, leftFrontPowerFactor);
        motorPowerFactors.put(leftBackDrive, leftBackPowerFactor);
        motorPowerFactors.put(rightFrontDrive, rightFrontPowerFactor);
        motorPowerFactors.put(rightBackDrive, rightBackPowerFactor);

        setMotorPowers(magLeft, magRight);

        telemetry.addData("front right power ", ((float)Math.round(rightFrontDrive.getPower()*100))/100);
        telemetry.addData("front left power ", ((float)Math.round(leftFrontDrive.getPower() *100))/100);
        telemetry.addData("back right power ", ((float)Math.round(rightBackDrive.getPower()*100))/100);
        telemetry.addData("back left power ", ((float)Math.round(leftBackDrive.getPower()*100))/100);
        telemetry.addData("magnitude left ", ((float)Math.round(magLeft*100))/100);
        telemetry.addData("thetaLeft ", ((float)Math.round(thetaLeft/pi*100))/100);

        telemetry.update();

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        boolean isAccelerating = true, isDecelerating = false;
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {

            if(isAccelerating){
                setMotorPowers(Math.max(getFractionalPosition(leftFrontDrive)*magLeft*20, 0.1), Math.max(getFractionalPosition(rightFrontDrive)*magRight*20, 0.1));
                if(getFractionalPosition(leftFrontDrive) >= .2){
                    isAccelerating = false;
                }
            }else if(!isDecelerating){
                setMotorPowers(magLeft, magRight);
                if(getFractionalPosition(leftFrontDrive) >= .8){
                    isDecelerating = true;
                }
            }else{
                setMotorPowers(Math.max((1-getFractionalPosition(leftFrontDrive))*magLeft*20, 0.2),
                        Math.max((1-getFractionalPosition(rightFrontDrive))*magRight*20, 0.2));
            }

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", motorTargetPositions.get(leftFrontDrive),  motorTargetPositions.get(rightFrontDrive));
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
            telemetry.addData("getFractionalPosition", getFractionalPosition(leftFrontDrive));
            telemetry.update();
        }
    }

    public void turnOnRunToPosition(){
        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnOffRunToPosition(){
        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopResetEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ceaseMotion(){
        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void setNewTargetPosition(double leftInches, double rightInches){

        // Determine new target position, and pass to motor controller
        motorInitialPositions.put(leftFrontDrive, leftFrontDrive.getCurrentPosition());
        motorInitialPositions.put(leftBackDrive, leftBackDrive.getCurrentPosition());
        motorInitialPositions.put(rightFrontDrive, rightFrontDrive.getCurrentPosition());
        motorInitialPositions.put(rightBackDrive, rightBackDrive.getCurrentPosition());

        motorTargetPositions.put(leftFrontDrive, motorInitialPositions.get(leftFrontDrive) + (int)(leftInches * COUNTS_PER_INCH));
        motorTargetPositions.put(leftBackDrive, motorInitialPositions.get(leftBackDrive) + (int)(leftInches * COUNTS_PER_INCH));
        motorTargetPositions.put(rightFrontDrive, motorInitialPositions.get(rightFrontDrive) + (int)(rightInches * COUNTS_PER_INCH));
        motorTargetPositions.put(rightBackDrive, motorInitialPositions.get(rightBackDrive) + (int)(rightInches * COUNTS_PER_INCH));

        leftFrontDrive.setTargetPosition(motorTargetPositions.get(leftFrontDrive));
        leftBackDrive.setTargetPosition(motorTargetPositions.get(leftBackDrive));
        rightFrontDrive.setTargetPosition(motorTargetPositions.get(rightFrontDrive));
        rightBackDrive.setTargetPosition(motorTargetPositions.get(rightBackDrive));
    }

    public double getFractionalPosition(DcMotor motor){
        return ((double)motor.getCurrentPosition() - motorInitialPositions.get(motor)) / (motorTargetPositions.get(motor) - motorInitialPositions.get(motor));
    }

    public void setMotorPowers(double magLeft, double magRight){
        leftFrontDrive.setPower(motorPowerFactors.get(leftFrontDrive) * magLeft);
        rightFrontDrive.setPower(motorPowerFactors.get(leftBackDrive) * magRight);
        leftBackDrive.setPower(motorPowerFactors.get(rightFrontDrive) * magLeft);
        rightBackDrive.setPower(motorPowerFactors.get(rightBackDrive) * magRight);
    }
}
