package org.firstinspires.ftc.other;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.Console;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "Odometry OpMode")
@Disabled
public class Odometry extends LinearOpMode {
    //Drive motors
    DcMotor frMotor, flMotor, brMotor, blMotor, wheel, launcherR, launcherL;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //final double COUNTS_PER_INCH = 307.699557;
    final double COUNTS_PER_REV = 8192; // CPR for REV Through Bore Encoders
    final double WHEEL_DIAMETER = 1.49606; //in inches, 38mm for odometry aluminum omni wheels
    double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER * 3.1415);

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontright", rbName = "backright", lfName = "frontleft", lbName = "backleft";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePositionOther globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePositionOther(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75, 111, 8.5, 0.0);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());


            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

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
        while (opModeIsActive() && distance > allowableDistanceError) {
             distance = Math.hypot(distanceToXTarget, distanceToYTarget);
             distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
             distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double robotMovmentYComponent = calculateY(robotMovementAngle - globalPositionUpdate.returnOrientation(), robotPower);
            double pivotCorrection = (desiredRobotOrientation - globalPositionUpdate.returnOrientation())*pivotCorrectionAdj;
           //double[] powers = {robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent, robotMovmentYComponent};//array for powers
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
            telemetry.addData("Pivot Correction: ", pivotCorrection);
            telemetry.update();
        }
    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        frMotor = hardwareMap.dcMotor.get(rfName);
        flMotor = hardwareMap.dcMotor.get(lfName);
        brMotor = hardwareMap.dcMotor.get(rbName);
        blMotor = hardwareMap.dcMotor.get(lbName);
        //Log.i("tobor", "done with get" );
        verticalLeft = hardwareMap.dcMotor.get("backright");
       verticalRight = hardwareMap.dcMotor.get("frontleft");
       horizontal = hardwareMap.dcMotor.get("frontright");
        Log.i("tobor1", "done with get" );

        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
}