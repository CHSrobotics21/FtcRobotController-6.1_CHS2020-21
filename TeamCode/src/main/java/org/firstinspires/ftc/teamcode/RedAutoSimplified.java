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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;


/**
 In auto, using goToPosition, robot will go as close to the lower leftmost corner that it would be able to reach
 without getting stuck in loop before obtaining second wobble goal. After the robot is there, the robot will
 run completely into the corner using a timer and strafing (change so it can account for bounce back) and recall
 the constructor and reset encoder values. Then, it will go to get the wobble goal using premeasured coordinates
 because it will now be accurate at that point. For box C, this movement may take too much time but delivering the
 wobble goal will be a little more accurate. Also, if this movement takes too long, change the end of auto parking
 for this box by moving straight back from the box using the current y position or adding a tape measure park system.
 Also, this system may not work for when we are not remote because the robot will be farther away from the corners and
 it will take more time to reset the odometry values so this idea is a temporary fix with a permanent fix maybe being
 an algorithm that accounts for change over time and/or new housing for the odometry wheels.
 Or another temporary fix is to run into the right wall before going to powershots to make sure they are accurate and
 using a color sensor go behind the white line and confirm your y position then reset encoders and recall constructor with premeasured
 points for that spot and use go to positions after that for powershots and wobble goal.
 */
@Autonomous(name = "red auto v6")
@Disabled
public class RedAutoSimplified extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS), powershotTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS), strafeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    String box;
    int x =-6;
    HardwareMapClass robo= new HardwareMapClass();

    File TeleOpStartingPos = AppUtil.getInstance().getSettingsFile("TeleOpStartingPos.txt");
    OdometryGlobalCoordinatePosition globalPositionUpdate;


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
        robo.init(hardwareMap);
        robo.ringStopper.setPosition(1);


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
                           //launchPower=(volts.getVoltage());
                       }

                   }
                   telemetry.addData("box: ",box);
                   telemetry.update();
               }
           }
       }
        timer.reset();


        if (opModeIsActive()) { // Linear OpMode
            robo.globalPositionUpdate = new OdometryGlobalCoordinatePosition(robo.verticalLeft, robo.verticalRight, robo.horizontal, robo.COUNTS_PER_INCH, 75, 111, 8.5, 0.0);
            robo.positionThread = new Thread(robo.globalPositionUpdate);
            robo.positionThread.start();

            robo.globalPositionUpdate.reverseRightEncoder();
            robo.globalPositionUpdate.reverseLeftEncoder();
            //globalPositionUpdate.reverseNormalEncoder();

            // starting postion for linear actuators
            robo.launcherAngle.setPosition(.43);
            robo.launcherAngleR.setPosition(.43);
            //hinge(true); testing

            //**GO TO BOX INSTRUCTIONS + DELIVER WOBBLE GOAL TO CORRECT BOX**
             goToBoxDeliverWobble(123,31,true, 0);
             sleep(1500);
            launch();
            sleep(1000);
            powershot();
            powershot();
            powershot();
            robo.collectorWheel.setPower(0);

            robo.launchSetZero();
             sleep(500);
            robo.goToPositionSetZero(60, 17, .6, 0, 4, opModeIsActive());// close to left corner
            timer.reset();
            while(strafeTimer.time()<2) //strafe into back wall to reset odometry
            {
                robo.frMotor.setPower(0);
                robo.flMotor.setPower(-.8);
                robo.brMotor.setPower(-.8);
                robo.blMotor.setPower(0);
            }
            robo.flMotor.setPower(0);
            robo.brMotor.setPower(0);
            robo.verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robo.verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robo.horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robo.verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robo.verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robo.horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robo.globalPositionUpdate = new OdometryGlobalCoordinatePosition(robo.verticalLeft, robo.verticalRight, robo.horizontal, robo.COUNTS_PER_INCH, 75, 54, 8.5, 0);

//             goToPositionSlowDown(76, 17, .4, 0, 1);
            robo.goToPositionSlowDown(81,17,.4,0,opModeIsActive());
             robo.grip(true);
             sleep(750);
             goToBoxDeliverWobble(76.5, 61, false, 6);
             sleep(750);
             int x = (int)robo.globalPositionUpdate.returnXCoordinate();
            robo.goToPositionSetZero(x,80,.9,0,2, opModeIsActive());//parking on white line

            String ContentsToWriteToFile = (robo.globalPositionUpdate.returnXCoordinate()/robo.COUNTS_PER_INCH) + " " + (robo.globalPositionUpdate.returnYCoordinate()/robo.COUNTS_PER_INCH) + " " + (robo.globalPositionUpdate.returnOrientation());
            ReadWriteFile.writeFile(TeleOpStartingPos, ContentsToWriteToFile);

            robo.goToPositionSlowDown(111, 24, .6, 0, opModeIsActive()); // go back to starting position for programmers testing ease :)


        }
        if (tfod != null) { //stop button
            tfod.shutdown();
        }
        robo.globalPositionUpdate.stop();
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

    public void hinge(boolean p)
    {
        if (p)// bring down arm
        {
            while(opModeIsActive()&&!(robo.brMotor.getCurrentPosition()>-2900&&robo.brMotor.getCurrentPosition()<-2700)) /*find threshold for when wobble goal arm is not down; don't want arm to be all the way down*/ {
                robo.wobbleArmHingeL.setPower(-1);
                robo.wobbleArmHingeR.setPower(1);
                telemetry.addData("Wobble counts", robo.brMotor.getCurrentPosition());
                telemetry.update();
            }
            robo.wobbleArmHingeL.setPower(0) ;
            robo.wobbleArmHingeR.setPower(0);
        }
        else {//bring arm back up
            while(opModeIsActive()&&!(robo.brMotor.getCurrentPosition()<-500&&robo.brMotor.getCurrentPosition()>-900)) /*find threshold for when wobble goal arm is not up*/ {
                robo.wobbleArmHingeL.setPower(1);
                robo.wobbleArmHingeR.setPower(-1);
            }
            robo.wobbleArmHingeL.setPower(0);
            robo.wobbleArmHingeR.setPower(0);
        }
    }
    public void launch(){
        robo.launcherL.setVelocity(500);
        robo.launcherR.setVelocity(-475);
    }
    public void goToBoxDeliverWobble(double goAroundRingsCoorX, double goAroundRingsCoorY, boolean doHingeArm, int comeback) {
        if (box == "a") {
            robo.goToPositionSlowDown(115-comeback, 79+comeback, .85, 0, opModeIsActive());// box a
        }
        else if (box == "b" || box == "c") {
            robo.goToPositionSetZero(goAroundRingsCoorX, goAroundRingsCoorY, .85, 0, 8, opModeIsActive()); // First movement out of starting postition to strafe to the first box
                if (box == "b") {
                    robo.goToPositionSlowDown(102-comeback, 103+comeback, .7, 0, opModeIsActive());// box b
                } else {//box c
                    robo.goToPositionSlowDown(115-comeback, 124+comeback, .7, 0, opModeIsActive());
                }
            }
            if (doHingeArm) {
                hinge(true);
            } //hinge arm out to deliver
            robo.grip(false); //ungrip wobble goal to release and deliver

        }
    public void powershot(){
        telemetry.addData("ring sensor: ", robo.ringStopperSensor.getDistance(DistanceUnit.CM));
         x -= 4;
        robo.goToPositionSetZero(96, 66.5, .35, x, 1.5, opModeIsActive());// move to behind white Line and position in front of powershot for powershot 1
        powershotTimer.reset();
        if(x==-10) {
            while (opModeIsActive() && (robo.ringStopperSensor.getDistance(DistanceUnit.CM) < 4.7&&powershotTimer.time()<2)) {
                robo.collectorWheel.setPower(-.7);
                telemetry.addData("ring stopper distance1: ", robo.ringStopperSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        else{
            while(opModeIsActive() && (robo.ringStopperSensor.getDistance(DistanceUnit.CM)>4.7&&powershotTimer.time()<2)){
                robo.collectorWheel.setPower(-.7);
                telemetry.addData("ring stopper distance: ", robo.ringStopperSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            powershotTimer.reset();
            robo.collectorWheel.setPower(0);
            while (opModeIsActive() && (robo.ringStopperSensor.getDistance(DistanceUnit.CM) < 4.7&powershotTimer.time()<2)) {
                robo.collectorWheel.setPower(-.7);
                telemetry.addData("ring stopper distance: ", robo.ringStopperSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        robo.collectorWheel.setPower(0);
    }
}
