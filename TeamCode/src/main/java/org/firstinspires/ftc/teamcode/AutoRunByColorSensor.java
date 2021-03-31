/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoColorSensor", group = "Concept")
@Disabled
public class AutoRunByColorSensor extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    List<Recognition> updatedRecognitions;
    ColorSensor colorSensor;
    DcMotor frMotor, flMotor, brMotor, blMotor;
    String box;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUSkSu//////AAABmbCDZkCjMUZdqlgBwBf0R6QmC3soC8rFfreNCDvJb7mhs7v6sWWIDBGTsR+tQeD9bSVikOsd2FpCDCK5qtLAy1U9ZgJZYN5O1IY3tuB6mnInb759EdsgxKJJT4OVFT1+QnozHYvi5BFK+Fwke9UKEohiv7baoXoYZbwDnjkTz6t1b5lg1em2Ebk2KGP3jOKS7fJkjQACDxIH9ikJ8/ShRnhMzVYge98MMhNxNTGM6T4rrdeBXZrS/pHAW9xu0k846P0/njOAVxgxgUywkX3GbbyqRuqio2KsQX9qCu+bGGEh08moFoMGdcX91l2QzOMkF7zjfFvmZfW8Aeth3sCt2+KonhX5vGAxnMeec0WZ105Q";

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
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //variableNameForServo = hardwareMap.get(Servo.class, "servoConfigurationName");
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(3.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while(!isStarted()&& !isStopRequested()){
            if (tfod != null) { // checks for object
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //List: collection of data <what type of list object is going to be (Tensor Flow objects)>
                //updatedRecognitions: object of the name of list
                //tfod reference object, .getUpdatedRecognitions: method to update recognitions, loads into list
                if (updatedRecognitions != null) { // checks for existence of recognitions
                    telemetry.addData("# Object Detected", updatedRecognitions.size()); //tells driver station how many objects(rings) it sees
                    box = "a"; //To assume we don't see anything
                    // step through the list of recognitions and display boundary info.
                    int i = 0; //int type variable i is set to value zero as start of count
                    for (Recognition recognition : updatedRecognitions) { //iteration loop ex. hot dogs in a cart. takes recognitions and assigns until end of list
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel()); //label will be "single", "quad", or nothing
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop()); // highest part of object, and leftmost part of object location
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getRight()>200&& recognition.getRight()<1000 && recognition.getBottom()>200 && recognition.getBottom()<1000) {
                            //check within bounds, to recognize only some rings. (right location)
                            if (recognition.getLabel() == "Single") {
                                box = "b";

                            } else if (recognition.getLabel() == "Quad") {
                                box = "c";
                            } else {
                                box = "a";
                            }
                        }

                    }
                    telemetry.addData("box: ",box);
                    telemetry.update();
                }
            }
        }
        timer.reset();

        if (opModeIsActive()) { // Linear OpMode

            while (opModeIsActive() && timer.time() < 3)
            {
                frMotor.setPower(0);
                blMotor.setPower(0);
                flMotor.setPower(.7);
                brMotor.setPower(.7);
            }
            timer.reset();
            int counter = 0;
            while (opModeIsActive() && colorSensor.red()<120)
            {
                frMotor.setPower(.7);
                blMotor.setPower(.7);
                flMotor.setPower(.7);
                brMotor.setPower(.7);
            }
            timer.reset();
            frMotor.setPower(0);
            blMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            while (opModeIsActive() && timer.time() < 1)
            {

            }
            timer.reset();
            telemetry.update();
            if (box == "b") {
                while (opModeIsActive() && timer.time() < 3) {
                    frMotor.setPower(.7);
                    blMotor.setPower(.7);
                    flMotor.setPower(-.7);
                    brMotor.setPower(-.7);
                }
                timer.reset();

                int numRedLines = 0;
                int[] colorValues = {0, 0, 0, 0, 0};
                while (opModeIsActive() && numRedLines<1)
                {
                    colorValues[0] = colorValues[1];
                    colorValues[1] = colorValues[2];
                    colorValues[2] = colorValues[3];
                    colorValues[3] = colorValues[4];
                    colorValues[4] = colorSensor.alpha();
                    frMotor.setPower(.7);
                    blMotor.setPower(.7);
                    flMotor.setPower(.7);
                    brMotor.setPower(.7);
                    if(IsValidLine(colorValues))
                    {
                        while(opModeIsActive()&&IsValidLine(colorValues))
                        {
                            colorValues[0] = colorValues[1];
                            colorValues[1] = colorValues[2];
                            colorValues[2] = colorValues[3];
                            colorValues[3] = colorValues[4];
                            colorValues[4] = colorSensor.alpha();
                        }
                        numRedLines++;
                    }
                    telemetry.addData("Lines: ",numRedLines);
                    telemetry.addData("Red value: ", colorSensor.red());
                    telemetry.update();

                }
            }
            else
            {//box c
                int numRedLines = 0;
                int[] colorValues = {0, 0, 0, 0, 0};
                while (opModeIsActive() && numRedLines<2)
                {
                    colorValues[0] = colorValues[1];
                    colorValues[1] = colorValues[2];
                    colorValues[2] = colorValues[3];
                    colorValues[3] = colorValues[4];
                    colorValues[4] = colorSensor.alpha();
                    frMotor.setPower(.7);
                    blMotor.setPower(.7);
                    flMotor.setPower(.7);
                    brMotor.setPower(.7);
                    if(IsValidLine(colorValues))
                    {
                        while(opModeIsActive()&&IsValidLine(colorValues))
                        {
                            colorValues[0] = colorValues[1];
                            colorValues[1] = colorValues[2];
                            colorValues[2] = colorValues[3];
                            colorValues[3] = colorValues[4];
                            colorValues[4] = colorSensor.alpha();
                        }
                        numRedLines++;
                    }
                    telemetry.addData("Lines: ",numRedLines);
                    telemetry.addData("Red value: ", colorSensor.red());
                    telemetry.update();

                }
                telemetry.addData("Red value counter: ",numRedLines);
            }

            //drop wobble goal
            //variableNameForServo.setPosition(1);

            //move to behind white Line
            if (box=="b" || box =="c")
            {
                while(opModeIsActive()&& colorSensor.alpha()<500)
                {
                    frMotor.setPower(-.7);
                    blMotor.setPower(-.7);
                    flMotor.setPower(-.7);
                    brMotor.setPower(-.7);
                }
                timer.reset();
                while (opModeIsActive() && timer.time() <2) // move farther behind white line; could be written using odometry
                {
                    frMotor.setPower(-.7);
                    blMotor.setPower(-.7);
                    flMotor.setPower(-.7);
                    brMotor.setPower(-.7);
                }
                timer.reset();
                /*
                while(opModeIsActive()&&timer.time()<2){
                    launcherR.setPower(1);
                    launcherL.setPower(1);
                }


                */
            }
            //if(box=="a" )

            frMotor.setPower(0);
            blMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);

        }
        if (tfod != null) { //stop button
            tfod.shutdown();
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
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); //take out during competition; leave parentheses blank
        tfodParameters.minResultConfidence = 0.85f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private boolean isOnLineEdge(int[] lastReads) {
        return true;
    }
    private boolean IsValidLine (int[] colorValue){
        for (int i=0; i<colorValue.length; i ++){
            if (colorValue[i] > 280 || colorValue[i] < 210) {
                return false;
            }
        }
        return true;
    }
}
