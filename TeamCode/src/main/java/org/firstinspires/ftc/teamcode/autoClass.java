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

@Autonomous(name="actual stuff", group="Exercises")
//@Disabled
public class autoClass extends LinearOpMode
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
    private final static int revTicks = 177;
    private final double wheelDiamerter = 4;
    private final double pi = Math.PI;
    private final double wheelCircumference = wheelDiamerter * pi;

    //The value below shows how many ticks there are per inch
    private double ticksPerInch = revTicks/wheelCircumference;

    //outcomes
    boolean foundGold = false;
    int stage = 0;

    //Gyrostuff
    static Orientation angles;
    static double heading = 0.0;

    //Vuforia instansiation
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "ARMl4sr/////AAAAGW7XCTx7E0rTsT4i0g6I9E8IY/EGEWdA5QHmgcnvsPFeuf+2cafgFWlJht6/m4ps4hdqUeDgqSaHurLTDfSET8oOvZUEOiMYDq2xVxNDQzW4Puz+Tl8pOFb1EfCrP28aBkcBkDfXDADiws03Ap/mD///h0HK5rVbe3KYhnefc0odh1F7ZZ1oxJy+A1w2Zb8JCXM/SWzAVvB1KEAnz87XRNeaJAon4c0gi9nLAdZlG0jnC6bx+m0140C76l14CTthmzSIdZMBkIb8/03aQIouFzLzz+K1fvXauT72TlDAbumhEak/s5pkN6L555F28Jf8KauwCnGyLnePxTm9/NKBQ4xW/bzWNpEdfY4CrBxFoSkq";

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;


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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /**
         //////////////////////////////////////////////////////////////////////////////////////////////
         //////////////////////////////////////////////////////////////////////////////////////////////
           _____                                                       _          __  __  __  __  __
          / ____|                                                     | |        / _|/ _|/ _|/ _|/ _|
         | |  __ _   _ _ __ ___  ___  ___ ___  _ __   ___          ___| |_ _   _| |_| |_| |_| |_| |_
         | | |_ | | | | '__/ _ \/ __|/ __/ _ \| '_ \ / _ \        / __| __| | | |  _|  _|  _|  _|  _|
         | |__| | |_| | | | (_) \__ \ (_| (_) | |_) |  __/        \__ \ |_| |_| | | | | | | | | | |
         \_____|\__,  |_|  \___/|___/\___\___/| .__/ \___|        |___/\__|\__,_|_| |_| |_| |_| |_|
                  __/ |                       | |
                 |___ /                       |_|
         //////////////////////////////////////////////////////////////////////////////////////////////
         //////////////////////////////////////////////////////////////////////////////////////////////
         **/

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

        // make sure the imu gyro is calibrated before continuing.
        /**while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }*/

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.


        waitForStart();

        start();

        telemetry.addData("Mode", "running");
        telemetry.update();

//        sleep(1000);
        drive(.5,10);
        driveSide(.5,10);
//        drive(.7, 2);
//        if(detectGold()) {
//            drive(.7, 10);
//        } else {
//            imuTurn(.4,30);
//            if(detectGold()){
//                drive(.7, 10);
//            } else {
//                imuTurn(.4,-60);
//                drive(.7,10);
//            }
//        }

    }

    public boolean detectGold() {

            if (tfod != null) {
                tfod.activate();
            }
            boolean result = false;
            boolean detected = false;
            /**
             ////////////////////////////////////////////////////////////////////////////
             ////////////////////////////////////////////////////////////////////////////
             __      __   __          _                    _          __  __  __  __  __
             \ \    / / / _|         (_)                  | |        / _|/ _|/ _|/ _|/ _|
             \ \  / /  | |_ ___  _ __ _  __ _          ___| |_ _   _| |_| |_| |_| |_| |_
             \ \/ /| | |  _/ _ \|'__| |/ _` |        / __| __| | | |  _|  _|  _|  _|  _|
             \  /| |_| | || (_)| |  | | (_| |        \__ \ |_| |_| | | | | | | | | | |
             \/  \__,_|_| \___/|_|  |_|\__,_|        |___/\__|\__,_|_| |_| |_| |_| |_|
             ///////////////////////////////////////////////////////////////////////////
             ////////////////////////////////////////////////////////////////////////////
             **/
        do {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if(updatedRecognitions.size() == 1) {
                        detected = true;
                    }
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("object", recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            result = true;
                        }
                    }
                    telemetry.update();
                }
            }
        } while(!detected);
            if (tfod != null) {
                tfod.shutdown();
            }

        return result;
    }

    public void drive(double power,double inch){

        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topRight.setTargetPosition((int)(bottomRight.getCurrentPosition() + inch * ticksPerInch));

        topRight.setPower(power);


        while (topRight.isBusy()) {
            topLeft.setPower(power);
            bottomLeft.setPower(power);
            bottomRight.setPower(power);
            telemetry.addData("encoder:", topRight.getCurrentPosition());
            telemetry.update();
        }
        topLeft.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);
    }

    // goes left
    public void driveSide(double power, double inch) {
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topRight.setTargetPosition((int)(bottomRight.getCurrentPosition() + inch * ticksPerInch));

        topRight.setPower(power);


        while (topRight.isBusy()) {
            topLeft.setPower(-power);
            bottomLeft.setPower(power);
            bottomRight.setPower(-power);
            telemetry.addData("encoder:", topRight.getCurrentPosition());
            telemetry.update();
        }
        topLeft.setPower(0);
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
        while (opModeIsActive() && !onHeadingNegative(speed, angle, 0.1)) {
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

    //VUDORIA METHODS
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}