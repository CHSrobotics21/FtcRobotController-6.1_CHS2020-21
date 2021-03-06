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

package org.firstinspires.ftc.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "RedAutoV4", group = "TFOdometry")
@Disabled
public class RedAutoV4 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    /* Encoder Variables to use Counts per inch on Odometry and conveyance middle wheel */
    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 2.3622; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);

    final double CPRCollectorWheel = 288;
    final double CollectorWheelDiameter = 5;
    double CPICollectorWheel = CPRCollectorWheel/(CollectorWheelDiameter*3.1415);
    double launchPower = 0.0;

    List<Recognition> updatedRecognitions;
    DcMotor frMotor, flMotor, brMotor, blMotor, collectorWheel;
    String box;
    CRServo wobbleArmHingeL, wobbleArmHingeR;
    Servo launcherAngleR, launcherAngle, wobbleArmGripL, wobbleArmGripR;
    DcMotor verticalLeft, verticalRight, horizontal, launcherR, launcherL;
    DigitalChannel gripSwitch, armSwitch;
    VoltageSensor volts;
    DistanceSensor intakeDistanceSensor, outDistanceSensor;


    File TeleOpStartingPos = AppUtil.getInstance().getSettingsFile("TeleOpStartingPos.txt");
    OdometryGlobalCoordinatePositionOther globalPositionUpdate;






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
        initDriveHardwareMap();


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
            globalPositionUpdate = new OdometryGlobalCoordinatePositionOther(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 111, 8.5, 0.0);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            globalPositionUpdate.reverseRightEncoder();
            globalPositionUpdate.reverseLeftEncoder();
            //globalPositionUpdate.reverseNormalEncoder();

            // starting postion for linear actuators
            launcherAngle.setPosition(.4);
            launcherAngleR.setPosition(.4);

            //**GO TO BOX INSTRUCTIONS + DELIVER WOBBLE GOAL TO CORRECT BOX**
            goToBoxDeliverWobble(123,31,true);
            launcherR.setPower(-.65); //initiate launchers to be stopped whe you should stop shooting
            launcherL.setPower(.65);
            goToPositionSetZero(89, 70, .5, 0, 1);// move to behind white Line and position in front of powershot for powershot 1
            sleep(1000);
            moveCollectorWheel();
            sleep(1500);
            goToPositionSetZero(82, 70, .5, 0, 1); //powershot 2
            sleep(500);
            moveCollectorWheel();
            sleep(1500);
            goToPositionSetZero(75, 70, .5, 0, 1); //powershot 3
            sleep(750);
            moveCollectorWheel();
            sleep(1500);
            /*^end of powershot shooting^*/

            /*go back to get 2nd wobble goal (where we think it's gonna be- tolerance could interrupt movements) while leaving the wobble goal arm out
               so we can get a grip from the right side of the robot
               incorporate sensor to detect if the robot has actually gotten grip on the wobble goal
                inch foreward to make sure of grip
                 */
            launcherL.setPower(0);
            launcherR.setPower(0);
            sleep(750);
            wobbleArmHingeL.setPower(-1);
            wobbleArmHingeR.setPower(1);
            sleep(250);
            wobbleArmHingeL.setPower(0);
            wobbleArmHingeR.setPower(0);
            sleep(500);
            goToPositionSlowDown(74, 15, .5, 0, 8);
            goToPositionSetZero(86, 17, .5, 0, .5);
            grip(true);
            sleep(750);
            goToBoxDeliverWobble(76.5, 61, false);
            goToPositionSetZero(80,80,.9,0,2);//parking behind white


            String ContentsToWriteToFile = (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH) + " " + (globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH) + " " + (globalPositionUpdate.returnOrientation());

            ReadWriteFile.writeFile(TeleOpStartingPos, ContentsToWriteToFile);
            telemetry.addData("StartingPostionX", globalPositionUpdate.returnXCoordinate());
            telemetry.addData("StartingPostionY", globalPositionUpdate.returnYCoordinate());
            telemetry.addData("StartingOrientation", globalPositionUpdate.returnOrientation());

            //goToPositionSetZero(111, 20, .9, 0, 2); // go back to starting position for programmers testing ease :)


        }
        if (tfod != null) { //stop button
            tfod.shutdown();
        }
        globalPositionUpdate.stop();
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


    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ){
        targetXPosition *= COUNTS_PER_INCH;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;
        double blPower = 0; // motor speed
        double brPower = 0; // motor speed
        double flPower = 0; // motor speed
        double frPower = 0; // motor speed
        double pivotCorrectionAdj = .01; // constant to scale down pivot correction angle to work with setting powers for mecanum drive motors
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while (opModeIsActive() && distance > allowableDistanceError ) { //correct heading too
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double robotMovmentYComponent = calculateY(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation())*pivotCorrectionAdj;
            blPower = robotMovmentYComponent - robotMovmentXComponent + pivotCorrection;
            flPower = robotMovmentYComponent + robotMovmentXComponent + pivotCorrection;
            brPower = robotMovmentYComponent + robotMovmentXComponent - pivotCorrection;
            frPower = robotMovmentYComponent - robotMovmentXComponent - pivotCorrection;
            //set powers to motors to move
            double maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            if (Math.abs(maxMotorPower) > 1) {
                flPower = (flPower / maxMotorPower)*robotPower;
                frPower = (frPower / maxMotorPower) *robotPower;
                blPower = (blPower / maxMotorPower) *robotPower;
                brPower = (brPower / maxMotorPower)*robotPower;
            } else if(Math.abs(maxMotorPower) < .03) {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("XComponent: ", robotMovmentXComponent/.9);
            telemetry.addData("YComponent: ", robotMovmentYComponent/.9);
            telemetry.addData("vertical right", globalPositionUpdate.verticalRightEncoderWheelPosition);
            telemetry.addData("vertical left", globalPositionUpdate.verticalLeftEncoderWheelPosition);
            telemetry.addData("horizontal", globalPositionUpdate.normalEncoderWheelPosition);
            telemetry.addData("Pivot Correction: ", pivotCorrection);
            //telemetry.addData("Limit Switch Status: ", gripSwitch.getState());
            telemetry.update();
        }
    }


    private void initDriveHardwareMap(){

        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");

        collectorWheel = hardwareMap.dcMotor.get("wheel");

        collectorWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherL = hardwareMap.dcMotor.get("launcherL");
        launcherR = hardwareMap.dcMotor.get("launcherR");
        launcherAngle = hardwareMap.get(Servo.class, "ServoL");
        launcherAngleR = hardwareMap.get(Servo.class, "ServoR");

        verticalLeft = hardwareMap.dcMotor.get("backleft");
        verticalRight = hardwareMap.dcMotor.get("frontleft");
        horizontal = hardwareMap.dcMotor.get("frontright");

        wobbleArmGripL = hardwareMap.servo.get("GripL");
        wobbleArmGripR = hardwareMap.servo.get("GripR");
        wobbleArmHingeL = hardwareMap.crservo.get("HingeL");
        wobbleArmHingeR = hardwareMap.crservo.get("HingeR");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intake");
        outDistanceSensor = hardwareMap.get(DistanceSensor.class, "out");
        gripSwitch = hardwareMap.get(DigitalChannel.class, "gripSwitch");
        gripSwitch.setMode(DigitalChannel.Mode.INPUT);
        armSwitch = hardwareMap.get(DigitalChannel.class, "armSwitch");
        armSwitch.setMode(DigitalChannel.Mode.INPUT);

        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public void goToPositionSetZero(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ){
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError);
        frMotor.setPower(0);
        blMotor.setPower(0);
        flMotor.setPower(0);
        brMotor.setPower(0);
    }

    public void grip(boolean p)
    {
        if (p)
        {
            wobbleArmGripL.setPosition(1);
            wobbleArmGripR.setPosition(1);
        }
        else
        {
            wobbleArmGripL.setPosition(0);
            wobbleArmGripR.setPosition(0);
        }

    }
    public void hinge(boolean p)
    {
        if (p)// bring down arm
        {
            while(opModeIsActive()&&brMotor.getCurrentPosition()>-217) /*find threshold for when wobble goal arm is not down; don't want arm to be all the way down*/ {
                wobbleArmHingeL.setPower(-1);
                wobbleArmHingeL.setPower(1);
            }
            wobbleArmHingeL.setPower(0);
            wobbleArmHingeR.setPower(0);
        }
        else {//bring arm back up
            while(opModeIsActive()&&brMotor.getCurrentPosition()<2947) /*find threshold for when wobble goal arm is not up*/ {
                wobbleArmHingeL.setPower(1);
                wobbleArmHingeL.setPower(-1);
            }
            wobbleArmHingeL.setPower(0);
            wobbleArmHingeR.setPower(0);
        }
    }

    public void moveCollectorWheel()
    { //place after go to position statements to shoot at power shot
        double inches=0;
        while(opModeIsActive()&&outDistanceSensor.getDistance(DistanceUnit.CM)<0) { /*find threshold for when ring is in certain position*/
            inches+=.1; //change increment
            collectorWheel.setTargetPosition(collectorWheel.getCurrentPosition()- (int)(inches*CPICollectorWheel)); // enter encoder counts or inches you want to move times counts per inch FOR THIS WHEEL AND MOTORS
            collectorWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            collectorWheel.setPower(1);
        }
    }

    public void launch(){
        //set launcher wheels same
        launcherL.setTargetPosition(collectorWheel.getCurrentPosition());
        launcherR.setTargetPosition(collectorWheel.getCurrentPosition()); //could go by counts instead of inches because theoretically you want to be moving for an unknown amount of inches
        launcherL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherL.setPower(.7);
        launcherR.setPower(-.7);
    }

    public void goToBoxDeliverWobble(double goAroundRingsCoorX, double goAroundRingsCoorY, boolean doHingeArm) {
        if (box == "a") {
            goToPositionSlowDown(110, 79, .6, 0, 8);// box a
        }
        else if (box == "b" || box == "c") {
                goToPositionSetZero(goAroundRingsCoorX, goAroundRingsCoorY, .6, 0, 8); // First movement out of starting postition to strafe to the first box
                if (box == "b") {
                    goToPositionSlowDown(97, 103, .6, 0, 8);// box b
                } else {//box c
                    goToPositionSlowDown(110, 130, .6, 0, 8);
                }
            }
            if (doHingeArm) {
                hinge(true);
            } //hinge arm out to deliver
            grip(false); //ungrip wobble goal to release and deliver
            sleep(1500);
        }
    public void goToPositionSlowDown(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ){
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower,desiredRobotOrientation,8);
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower-.3,desiredRobotOrientation,1);
    }

}
