package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class HardwareMapClassLinearExtend extends LinearOpMode {
    HardwareMap hwMap=null;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    String box;
    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 2.3622; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);
    final int CPRCollectorWheel = 288, CollectorWheelDiameter = 5;
    double CPICollectorWheel = CPRCollectorWheel/(CollectorWheelDiameter*3.1415);
    boolean isGoToPosition = false, islaunchRunning= false;
    OdometryGlobalCoordinatePosition globalPositionUpdate = null;
    Thread positionThread = null;
    Orientation gyroAngles = null;
    public DcMotor frMotor=null, flMotor=null, brMotor=null, blMotor=null, collectorWheel=null, collector=null, verticalLeft=null, verticalRight=null, horizontal=null;
    public DcMotorEx launcherR=null, launcherL=null;
    public Servo launcherAngle=null, launcherAngleR=null, wobbleArmGripL=null, wobbleArmGripR=null, ringStopper=null;
    public CRServo wobbleArmHingeL=null, wobbleArmHingeR=null;
    public BNO055IMU imu=null;
    public DistanceSensor intakeDistanceSensor=null, ringStopperSensor=null;

    double desiredRobotHeading;
    int rotations = 0;


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
    public TFObjectDetector tfod;


    public HardwareMapClassLinearExtend(){}

    @Override
    public void runOpMode(){

    }

    public void initHwMap(HardwareMap ahwMap){
        hwMap = ahwMap;
//distanceColor = hwMap.colorSensor.get("distanceColor");
        frMotor = hwMap.dcMotor.get("frontright");
        flMotor = hwMap.dcMotor.get("frontleft");
        brMotor = hwMap.dcMotor.get("backright");
        blMotor = hwMap.dcMotor.get("backleft");
        collector = hwMap.dcMotor.get("collector");

        ringStopperSensor = hwMap.get(DistanceSensor.class,"ringStopperSensor");

        launcherR = hwMap.get(DcMotorEx.class,"launcherR");
        launcherL = hwMap.get(DcMotorEx.class,"launcherL");
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectorWheel = hwMap.dcMotor.get("wheel");
        launcherAngle = hwMap.get(Servo.class, "ServoL");
        launcherAngleR = hwMap.get(Servo.class, "ServoR");

        wobbleArmGripL = hwMap.servo.get("GripL");
        wobbleArmGripR = hwMap.servo.get("GripR");
        ringStopper = hwMap.servo.get("ringStopper");
        wobbleArmHingeL = hwMap.crservo.get("HingeL");
        wobbleArmHingeR = hwMap.crservo.get("HingeR");
        intakeDistanceSensor = hwMap.get(DistanceSensor.class, "intake");

        verticalLeft = hwMap.dcMotor.get("backleft");
        verticalRight = hwMap.dcMotor.get("frontleft");
        horizontal = hwMap.dcMotor.get("frontright");

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
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
        }
    }
    public void goToPositionSetZero(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError);
        frMotor.setPower(0);
        blMotor.setPower(0);
        flMotor.setPower(0);
        brMotor.setPower(0);
    }
    public void goToPositionSlowDown(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation){
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower,desiredRobotOrientation,8);
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower-.3,desiredRobotOrientation,1.2);
    }

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
    public void driveTrainZero(){
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
    public void launch(){
        launcherL.setVelocity(725);
        launcherR.setVelocity(-775);
        islaunchRunning = true;
    }
    public void launchSetZero(){
        launcherL.setVelocity(0);
        launcherR.setVelocity(0);
        islaunchRunning = false;
    }
    public void moveCollectorWheel()
    { //place after go to position statements to shoot at power sh
        collectorWheel.setTargetPosition(collectorWheel.getCurrentPosition() - (int)(3.5*CPICollectorWheel)); // enter encoder counts or inches you want to move times counts per inch FOR THIS WHEEL AND MOTORS
        collectorWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectorWheel.setPower(-1);
    }

    public double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }
        return (rotations * 360 + gyroAngles.firstAngle);
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
            wobbleArmHingeL.setPower(-1);
            wobbleArmHingeR.setPower(1);

        }
        else {//bring arm back up
            wobbleArmHingeL.setPower(1);
            wobbleArmHingeR.setPower(-1);
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
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
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); //take out during competition; leave parentheses blank
        tfodParameters.minResultConfidence = 0.85f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void collectorWheelRun(double power) {//simplify collectorwheel running commands while also ensuring power mode is correct
        collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorWheel.setPower(power);
    }
    public void boxDetection(){
        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(3.5, 1.78);
        }

        /** Wait for the game to begin */
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
    }
}
