package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

//lol
/**
 * Created by nihal on 1/17/18.
 */


@Autonomous(name="Red 1 Vuforia", group = "Tourney Codes")

public class RedOneVuforia extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor armMotor;
    //left
    private Servo handServo1;
    //right
    private Servo handServo2;
    private Servo colorServo;
    private ColorSensor colorSensor;


    //position values
    private final static int revTicks = 1120;
    private final static int gearValue = 2;
    private final double wheelDiamerter = 6;
    private final double pi = Math.PI;
    private final double wheelCircumference = wheelDiamerter * pi;
    private final int ELEVATOR_TOP_POSITION = -2900;
    private final int ELEVATOR_BOTTOM_POSITION = 0;

    //The value below shows how many ticks there are per inch
    private double ticksPerInch = revTicks/wheelCircumference;

    VuforiaLocalizer vuforia;
    private String tempVuf;

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARMl4sr/////AAAAGW7XCTx7E0rTsT4i0g6I9E8IY/EGEWdA5QHmgcnvsPFeuf+2cafgFWlJht6/m4ps4hdqUeDgqSaHurLTDfSET8oOvZUEOiMYDq2xVxNDQzW4Puz+Tl8pOFb1EfCrP28aBkcBkDfXDADiws03Ap/mD///h0HK5rVbe3KYhnefc0odh1F7ZZ1oxJy+A1w2Zb8JCXM/SWzAVvB1KEAnz87XRNeaJAon4c0gi9nLAdZlG0jnC6bx+m0140C76l14CTthmzSIdZMBkIb8/03aQIouFzLzz+K1fvXauT72TlDAbumhEak/s5pkN6L555F28Jf8KauwCnGyLnePxTm9/NKBQ4xW/bzWNpEdfY4CrBxFoSkq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes


        motorLeft = hardwareMap.dcMotor.get("motorTest2");
        motorRight = hardwareMap.dcMotor.get("motorTest");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        colorServo = hardwareMap.servo.get("armServo");
        handServo1 = hardwareMap.servo.get("servo1");
        handServo2 = hardwareMap.servo.get("servo2");
        colorSensor = hardwareMap.colorSensor.get("color");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);


        /**This is where everything begins**/


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        waitForStart();



/**        closeElevator();

        sleep(500);
        liftUp();

        redTeamJewel();
        sleep(1000);

        drive(.2,-15);
        sleep(3000);
**/
        relicTrackables.activate(); // Activate Vuforia

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addData("VuMark", "left");
            drive(.2,-10);
            sleep(1000);
            Turn(14);
            sleep(2000);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("VuMark", "right");
            drive(.2,-10);
            sleep(1000);
            Turn(22);
            sleep(2000);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("VuMark", "center");
            drive(.2,-10);
            sleep(1000);
            Turn(18);
            sleep(2000);

        }else {
            telemetry.addData("VuMark", "not visible");
        }
        sleep(1000);
        telemetry.update();
        sleep(10000);


/**        drive(.2,2);
        sleep(1000);

        openElevator();
        sleep(500);

        drive(.2,-2);
        stop();


**/
    }

    public void drive(double power,double inch){

        motorRight.setTargetPosition((int)(motorRight.getCurrentPosition() + inch * ticksPerInch));
        motorLeft.setTargetPosition((int)(motorLeft.getCurrentPosition() + inch * ticksPerInch));

        motorLeft.setPower(power);
        motorRight.setPower(power);

        while (motorRight.isBusy() && motorLeft.isBusy());
        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    public void Turn(int inch){
        //for 180 degrees inch = 23

        motorRight.setTargetPosition((int)(motorRight.getCurrentPosition() + inch * ticksPerInch));
        motorLeft.setTargetPosition((int)(motorLeft.getCurrentPosition() - inch * ticksPerInch));
        motorLeft.setPower(0.2);
        motorRight.setPower(0.2);
        while (motorRight.isBusy() && motorLeft.isBusy());
    }

    public void resetEncoders(){
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnLeft(double inch){
        motorRight.setTargetPosition((int)(motorRight.getCurrentPosition() + inch * ticksPerInch));

        motorRight.setPower(.1);
        while (motorRight.isBusy() && motorLeft.isBusy());
    }

    public void turnRight(double inch){
        motorLeft.setTargetPosition((int)(motorLeft.getCurrentPosition() + inch * ticksPerInch));

        motorLeft.setPower(.1);
    }

    public void closeElevator(){
        handServo1.setPosition(0);
        handServo2.setPosition(1);
    }

    public void openElevator(){
        handServo1.setPosition(.5);
        handServo2.setPosition(.2);
    }

    public void liftUp(){
        armMotor.setTargetPosition(ELEVATOR_TOP_POSITION/3);
        armMotor.setPower(.5);
        while(armMotor.isBusy());
    }
    public void setDown(){
        armMotor.setTargetPosition(ELEVATOR_BOTTOM_POSITION);
        armMotor.setPower(.5);
        while(armMotor.isBusy());
    }

    public void knockOutRedJewel(){
        drive(1, -3);
        colorServo.setPosition(0);
        colorSensor.enableLed(false);
        drive(.1,3);
    }

    public void knockOutBlueJewel(){
        drive(1, 3);
        colorServo.setPosition(0);
        colorSensor.enableLed(false);
        drive(.1,-3);
    }

    public void redTeamJewel(){
        colorServo.setPosition(1);
        sleep(2000);
        colorSensor.enableLed(true);

        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.update();

        if(colorSensor.red() > colorSensor.blue()){
            knockOutBlueJewel();
        }
        else{
            knockOutRedJewel();
        }
    }
    public void getVufPos(){

    }
}