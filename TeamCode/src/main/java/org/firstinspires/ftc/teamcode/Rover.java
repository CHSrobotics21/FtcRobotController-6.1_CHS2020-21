/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Summer Programming")
//@Disabled
public class Rover extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    double flPower, frPower, blPower, brPower;
    DcMotor flMotor, frMotor, brMotor, blMotor;
    double desiredHeading, maxMotorPower;
    BNO055IMU imu;
    Orientation gyroAngles;
    boolean robotPerspective = false;
    double fieldReference, y, x, joystickAngle, joystickAngle360, driveSpeed, driveRotation, objectAngle;
    int rotations;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string
     * from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AUSkSu//////AAABmbCDZkCjMUZdqlgBwBf0R6QmC3soC8rFfreNCDvJb7mhs7v6sWWIDBGTsR+tQeD9bSVikOsd2FpCDCK5qtLAy1U9ZgJZYN5O1IY3tuB6mnInb759EdsgxKJJT4OVFT1+QnozHYvi5BFK+Fwke9UKEohiv7baoXoYZbwDnjkTz6t1b5lg1em2Ebk2KGP3jOKS7fJkjQACDxIH9ikJ8/ShRnhMzVYge98MMhNxNTGM6T4rrdeBXZrS/pHAW9xu0k846P0/njOAVxgxgUywkX3GbbyqRuqio2KsQX9qCu+bGGEh08moFoMGdcX91l2QzOMkF7zjfFvmZfW8Aeth3sCt2+KonhX5vGAxnMeec0WZ105Q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        String position = "";
        if (tfod != null) {
            tfod.activate();
        }
        frMotor = hardwareMap.dcMotor.get("frontright");
        brMotor = hardwareMap.dcMotor.get("backright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        blMotor = hardwareMap.dcMotor.get("backleft");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while (!isStarted()&& !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        objectAngle = -recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        telemetry.addData("angle " + recognition.getLabel(), objectAngle);
                        telemetry.addData("object height :", recognition.getHeight());

                    }
                    //rectangle frame is two objects grouped so move robot to read different
                    //choose object based on a.) confidence,
                    gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.update();

                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        if(opModeIsActive()) {
            angleRobot(objectAngle);

//            desiredHeading = getIntegratedHeading();
//            gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            driveRotation = (desiredHeading - getIntegratedHeading()) / (Math.abs(desiredHeading - getIntegratedHeading())) * .05;
//            if (robotPerspective) { //Controls are mapped to the robot perspective
//                fieldReference = 0;
//                //Positive values for x axis are joystick right
//                //Positive values for y axis are joystick down
//                y = Range.clip(-gamepad1.right_stick_y,-1,1);
//                x = Range.clip(-gamepad1.right_stick_x,-1,1);
//                joystickAngle = Math.atan2(x,y);
//            } else {   //Controls are mapped to the field
//                fieldReference = desiredHeading;
//                //Positive values for x axis are joystick right
//                //Positive values for y axis are joystick down
//                y = Range.clip(gamepad1.right_stick_x,-1,1);
//                x = Range.clip(-gamepad1.right_stick_y,-1,1);
//                joystickAngle = Math.atan2(x,y);
//                // joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
//            }
//            joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
//            driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);
//
//            flPower = Math.cos(Math.toRadians(joystickAngle360 - desiredHeading) + Math.PI / 4);
//            frPower = Math.sin(Math.toRadians(joystickAngle360 - desiredHeading) + Math.PI / 4);
//            blPower = Math.sin(Math.toRadians(joystickAngle360 - desiredHeading) + Math.PI / 4);
//            brPower = Math.cos(Math.toRadians(joystickAngle360 - desiredHeading) + Math.PI / 4);
//
//            maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));
//
//            //Ratio the powers for direction
//            flPower = flPower / maxMotorPower;
//            frPower = frPower / maxMotorPower;
//            blPower = blPower / maxMotorPower;
//            brPower = brPower / maxMotorPower;
//
//            flPower = driveSpeed * flPower - driveRotation;
//            frPower = driveSpeed * frPower + driveRotation;
//            blPower = driveSpeed * blPower - driveRotation;
//            brPower = driveSpeed * brPower + driveRotation;
//
//            maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));
//
//
//            if (Math.abs(maxMotorPower) > 1) {
//                flPower = flPower / maxMotorPower;
//                frPower = frPower / maxMotorPower;
//                blPower = blPower / maxMotorPower;
//                brPower = brPower / maxMotorPower;
//           } //else if (Math.abs(maxMotorPower) < .03) {
////                flPower = 0;
////                frPower = 0;
////                blPower = 0;
////                brPower = 0;
////            }
//
//            flMotor.setPower(flPower);
//            frMotor.setPower(frPower);
//            blMotor.setPower(blPower);
//            brMotor.setPower(brPower);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void angleRobot(double angleOfObject){
        //double targetAngle = getIntegratedHeading()+angleAdd;
//        boolean isNeg = false;
//        if(angleOfObject<0)
//        {
//            angleOfObject = Math.abs(angleOfObject);
//            isNeg = true;
//        }
        while(opModeIsActive()&&angleOfObject > getRoboAngle()){
            gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            brMotor.setPower(-.7);
            blMotor.setPower(.7);
//            if(isNeg){
//                frMotor.setPower(-.7);
//                flMotor.setPower(.7);
//            }
//            else
//            {
//                frMotor.setPower(.7);
//                flMotor.setPower(-.7);
//            }

            telemetry.addData("IMU angle", getRoboAngle());
            telemetry.update();
        }
        frMotor.setPower(0);
        flMotor.setPower(0);

    }
    private double getRoboAngle() {
        if(desiredHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }
}