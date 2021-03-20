package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Shared.DriveOBJ;
//import org.firstinspires.ftc.teamcode.Drive;

import java.util.List;

@Autonomous(name = "Competition OBJ", group = "Autonomous")
//@Disabled
public class CompetitionAutonomousOBJ extends LinearOpMode {
    private final String TAG = getClass().getName();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private ElapsedTime runtime = new ElapsedTime();

    private Servo Load;
    private DcMotorEx Ring;
    private DcMotor Elevate, Sweep;

    private static final String VUFORIA_KEY =
            "AUmiib//////AAABmbrFAeBqbkd/hmTwBoU6jXFUjeYK8xAgeYu6r9ZuLmpgb4tqNl4oIhXkpbBXmCusnhPlxJ3DHEkExTnQKhvCU49Yu2jslI6vaQ+V5F21ZAbbBod6lm9zyBEpkujo7IOq2TdOaJSIdN5wW3zxrHTksfrzBuKZKRsArompruh7jrm/B4W3F/EunA8ymkVoi29W84q81XMwJyonWlS2sd3pebXvLW0YOKmA63QgdmtSpp9XVAccwiH8ND8rk7FXlIIucim1Ig5FmVPLIx88t7doptXh8uiXfHHMqXc1T1MrRvfemYaUqyg7I5lYLNjLhuRmBZO3BM/qoyjPhpMVtGNh6+z3VgaKhP7O6zI07W0mmMfO";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        Load = hardwareMap.get(Servo.class, "Load");
        Ring = hardwareMap.get(DcMotorEx.class, "Ring");
        Elevate = hardwareMap.get(DcMotor.class, "Elevate");
        Sweep = hardwareMap.get(DcMotor.class, "Sweep");

        Ring.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(1.75, 1.78);
        }

        Log.i(TAG, "runOpmode: Waiting for IMU...");

        DriveOBJ drive = new DriveOBJ(this);
        drive.init();

        telemetry.addLine("IMU ready");
        telemetry.update();
        Log.i(TAG, "runOpmode: IMU ready");

        waitForStart();

        drive.turnSweeper(.3, 1);

        runtime.reset();
        int singleVotes = 0;
        int quadVotes = 0;
        int votes = 0;
        long startMillis = System.currentTimeMillis();

        while (System.currentTimeMillis() - startMillis < 2000) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    votes++;
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getConfidence() < 0.3) {
                            continue;
                        }
                        if (recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                            quadVotes++;
                        } else {
                            singleVotes++;
                        }
                    }
                }
            }
            sleep(100);
        }

        telemetry.addData("votes (single, quad)", "%d, %d", singleVotes, quadVotes);
        telemetry.addData("total votes:", votes);
        telemetry.addData("vote ratios (single, quad)", "%.3f, %.3f", singleVotes/(double)votes, quadVotes/(double)votes);
        telemetry.addLine("Waiting for IMU...");
        telemetry.update();
        Log.i(TAG, String.format("runOpMode: votes (single, quad) %d, %d", singleVotes, quadVotes));
        Log.i(TAG, String.format("total votes", votes));
        Log.i(TAG, String.format("vote ratios (single, quad)", "%.3f, %.3f", singleVotes/(double)votes, quadVotes/(double)votes));

        telemetry.addData("votes (single, quad)", "%d, %d", singleVotes, quadVotes);
        telemetry.addData("total votes:", votes);
        telemetry.addData("vote ratios (single, quad)", "%.3f, %.3f", singleVotes/(double)votes, quadVotes/(double)votes);


        drive.runUsingEncoder();

        Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");

        double SPEED = 1;
        if(singleVotes/(double)votes > 0.3){
            //drive to square b
            drive.vroomVroomMonitorTicks(SPEED, 16, 18, 10);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, 0, 20, 10);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, -30, 32, 10);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, 0, 12, 10);
            drive.ceaseMotion();
            sleep(150);
            drive.turnSweeper(.7, 1.0);
            sleep(1000);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, 0, -33, 10);
            //sleep(1000);  //Need to keep sweeper still until wobble goal comes to rest. Remove this when there is more code below
        }else if(quadVotes/(double)votes > 0.3){
            //drive to square c
            drive.vroomVroomMonitorTicks(SPEED, 16, 13, 10);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, 0, 89, 10);
            drive.ceaseMotion();
            sleep(150);
            drive.turnSweeper(.7, 1.0);
            sleep(1000);  //Need to keep sweeper still until wobble goal comes to rest. Remove this when there is more code below
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, -27, -46, 4);
        }else{
            //drive to square a
            drive.vroomVroomMonitorTicks(SPEED, 16, 16, 10);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, 0, 34, 10);
            drive.ceaseMotion();
            sleep(150);
            drive.turnSweeper(.7, 1.0);
            Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            drive.vroomVroomMonitorTicks(SPEED/2, -27, -14, 4);
            drive.vroomVroomMonitorTicks(SPEED/2, 0, 8, 4);
            // Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            // drive.vroomVroomMonitorTicks(SPEED/2, 0, -12, 4);
            // Log.i("DriveByEncoderOpMode", "************************ xyzabc *****************************");
            // drive.vroomVroomMonitorTicks(SPEED/2, 0, 12, 4);
        }
        drive.ceaseMotion();

        Ring.setVelocity(280, AngleUnit.DEGREES);
        //Ring.setPower(.95);
        sleep(1000);

        Load.setPosition(0.67);
        sleep(900);
        Load.setPosition(0);
        sleep(1000);

        Load.setPosition(0.67);
        sleep(900);
        Load.setPosition(0);
        sleep(1000);

        Load.setPosition(0.67);
        sleep(900);
        Load.setPosition(0);
        sleep(10);

        drive.vroomVroomMonitorTicks(SPEED/2, 0, 4, 10);

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
