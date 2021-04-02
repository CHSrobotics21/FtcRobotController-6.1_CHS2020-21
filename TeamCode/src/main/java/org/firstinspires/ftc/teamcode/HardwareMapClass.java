package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class HardwareMapClass {
    HardwareMap hwMap=null;
    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 2.3622; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);
    final int CPRCollectorWheel = 288, CollectorWheelDiameter = 5;
    double CPICollectorWheel = CPRCollectorWheel/(CollectorWheelDiameter*3.1415);
    boolean isGoToPosition = false, islaunchRunning= false;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;
    Orientation gyroAngles;
    public DcMotor frMotor=null, flMotor=null, brMotor=null, blMotor=null, collectorWheel=null, collector=null, verticalLeft=null, verticalRight=null, horizontal=null;
    public DcMotorEx launcherR=null, launcherL=null;
    public Servo launcherAngle=null, launcherAngleR=null, wobbleArmGripL=null, wobbleArmGripR=null, ringStopper=null;
    public CRServo wobbleArmHingeL=null, wobbleArmHingeR=null;
    public BNO055IMU imu=null;
    public DistanceSensor intakeDistanceSensor=null, ringStopperSensor=null;

    double desiredRobotHeading;
    int rotations = 0;

    public HardwareMapClass(){}

    public void init(HardwareMap ahwMap){
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
    /*
    Go to position for iterative
     */
    public boolean goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ) {
        targetXPosition *= COUNTS_PER_INCH;
        isGoToPosition = true;
        targetYPosition *= COUNTS_PER_INCH;
        allowableDistanceError *= COUNTS_PER_INCH;
        double blPower = 0, brPower = 0, flPower, frPower = 0; // motor speed
        double pivotCorrectionAdj = .01; // constant to scale down pivot correction angle to work with setting powers for mecanum drive motors
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        if (distance > allowableDistanceError) { //correct heading too
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double robotMovmentYComponent = calculateY(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation()) * pivotCorrectionAdj;
            //double[] powers = {robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent};//array for powers
            blPower = robotMovmentYComponent - robotMovmentXComponent + pivotCorrection;
            flPower = robotMovmentYComponent + robotMovmentXComponent + pivotCorrection;
            brPower = robotMovmentYComponent + robotMovmentXComponent - pivotCorrection;
            frPower = robotMovmentYComponent - robotMovmentXComponent - pivotCorrection;
            //set powers to motors to move
            double maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            if (Math.abs(maxMotorPower) > 1) {
                flPower = (flPower / maxMotorPower) * robotPower;
                frPower = (frPower / maxMotorPower) * robotPower;
                blPower = (blPower / maxMotorPower) * robotPower;
                brPower = (brPower / maxMotorPower) * robotPower;
            } else if (Math.abs(maxMotorPower) < .03) {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
            isGoToPosition = true;
            return false;
        }
        else{
            driveTrainZero();
            isGoToPosition = false;
            return true;
        }
    }
    public void goToPositionSlowDown(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError ){
        if (goToPosition(targetXPosition,targetYPosition,robotPower,desiredRobotOrientation,8)) {}
        if (goToPosition(targetXPosition,targetYPosition,robotPower-.3,desiredRobotOrientation,1)) {}

    }
    /*
    go to position for linear
     */
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, boolean isLinear ){
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
        while (isLinear && distance > allowableDistanceError ) { //correct heading too
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
    public void goToPositionSetZero(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError, boolean isLinear ){
        goToPosition(targetXPosition, targetYPosition, robotPower, desiredRobotOrientation, allowableDistanceError, isLinear);
        frMotor.setPower(0);
        blMotor.setPower(0);
        flMotor.setPower(0);
        brMotor.setPower(0);
    }
    public void goToPositionSlowDown(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, boolean isLinear ){
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower,desiredRobotOrientation,8, isLinear);
        goToPositionSetZero(targetXPosition,targetYPosition,robotPower-.3,desiredRobotOrientation,1.2, isLinear);
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
    public void collectorWheelRun(double power){//simplify collectorwheel running commands while also ensuring power mode is correct
        collectorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorWheel.setPower(power);
    }
}
