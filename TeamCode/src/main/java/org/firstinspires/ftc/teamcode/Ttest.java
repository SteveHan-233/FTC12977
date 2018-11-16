// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Autonomous(name="SDd", group="Exercises")
//@Disabled
public class Ttest extends LinearOpMode
{
    //normal instansiation
    private DcMotor bottomLeft;
    private DcMotor bottomRight;
    private DcMotor topLeft;
    private DcMotor topRight;
    private static BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //encoder info
    private final static int revTicks = 288;
    private final double wheelDiamerter = 4;
    private final double pi = Math.PI;
    private final double wheelCircumference = wheelDiamerter * pi;

    //The value below shows how many ticks there are per inch
    private double ticksPerInch = revTicks/wheelCircumference;

    //Gyrostuff
    static Orientation angles;
    static double heading = 0.0;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");
        bottomRight = hardwareMap.dcMotor.get("bottomRight");
        topLeft = hardwareMap.dcMotor.get("topLeft");
        topRight = hardwareMap.dcMotor.get("topRight");

        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bottomRight.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.REVERSE);

        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();




        waitForStart();

        start();

        imuTurn(.4, 90);



        // turn the motors off.
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        topLeft.setPower(0);
    }


    public void drive(double power,double inch){

        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bottomRight.setTargetPosition((int)(bottomRight.getCurrentPosition() + inch * ticksPerInch));
        bottomLeft.setTargetPosition((int)(bottomLeft.getCurrentPosition() + inch * ticksPerInch));

        bottomLeft.setPower(power);
        bottomRight.setPower(power);



        while (bottomRight.isBusy() && bottomLeft.isBusy());
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

    }

    public void imuTurn (  double speed, double angle) {

        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.addData("DD",heading);
            telemetry.update();
        }

    }

    public void imuTurn2 (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeadingNegative(speed, angle, 5)) {
            // Update telemetry & Allow time for other processes to run.

            telemetry.addData("DD",heading);
            telemetry.update();
        }

    }

    boolean onHeading(double speed, double angle, double PCoeff) {

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        returnAngles();

        bottomLeft.setPower(-speed);
        topLeft.setPower(-speed);
        bottomRight.setPower(speed);
        topRight.setPower(speed);

        if(heading > angle - 5){
            onTarget = true;
            bottomLeft.setPower(0);
            topLeft.setPower(0);
            bottomRight.setPower(0);
            topRight.setPower(0);

        }

        return onTarget;
    }

    boolean onHeadingNegative(double speed, double angle, double PCoeff) {

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        returnAngles();

        bottomLeft.setPower(-speed);
        topLeft.setPower(-speed);
        bottomRight.setPower(-speed);
        topRight.setPower(-speed);

        if(heading < angle){
            onTarget = true;
            bottomLeft.setPower(0);
            topLeft.setPower(0);
            bottomRight.setPower(0);
            topRight.setPower(0);

        }

        return onTarget;
    }

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public static void returnAngles(){
        //starting IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return necessary angle
        heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }


}