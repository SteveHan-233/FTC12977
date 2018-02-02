package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Tele Op")
public class TeleOpMode extends LinearOpMode{

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor armMotor;
    private DcMotor relicMotor;
    private Servo relicServo1;
    private Servo relicServo2;
    private Servo relicGrabber;
    //left
    private Servo handServo1;
    //right
    private Servo handServo2;
    private double speedFactor = 0;
    private Servo colorServo;
    private boolean temp = false;

    private final int ELEVATOR_TOP_POSITION = -2930;
    private final int ELEVATOR_BOTTOM_POSITION = 0;
    private final int RELIC_EXTENDED_POSITION = -2600;
    private final int RELIC_CONTRACTED_POSITION = 50;


    public void runOpMode() throws InterruptedException {

        motorLeft = hardwareMap.dcMotor.get("motorTest2");
        motorRight = hardwareMap.dcMotor.get("motorTest");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        colorServo = hardwareMap.servo.get("armServo");
        handServo1 = hardwareMap.servo.get("servo1");
        handServo2 = hardwareMap.servo.get("servo2");
        relicServo1 = hardwareMap.servo.get("relicServo1");
        relicServo2 = hardwareMap.servo.get("relicServo2");
        relicGrabber = hardwareMap.servo.get("relicGrabber");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {

            relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.a)
                colorServo.setPosition(.3);
            if(gamepad1.b)
                colorServo.setPosition(.7);

            if (gamepad2.dpad_up && !gamepad2.right_bumper) {
                armMotor.setTargetPosition(ELEVATOR_TOP_POSITION);
                armMotor.setPower(1);
            }
            if (gamepad2.dpad_down && !gamepad2.right_bumper) {
                armMotor.setTargetPosition(ELEVATOR_BOTTOM_POSITION);
                armMotor.setPower(1);
            }
            if (gamepad2.dpad_left && !gamepad2.right_bumper) {
                armMotor.setTargetPosition(ELEVATOR_TOP_POSITION/3);
                armMotor.setPower(1);
            }
            if (gamepad2.dpad_right && !gamepad2.right_bumper) {
                armMotor.setTargetPosition(ELEVATOR_TOP_POSITION*2/3);
                armMotor.setPower(1);
            }


            while (gamepad2.right_stick_button) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(gamepad2.right_stick_y);
                if (!gamepad2.right_stick_button) {
                    armMotor.setPower(0);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }



            int position = armMotor.getCurrentPosition();
            telemetry.addData("Elevator Encoder Position", position);
            //0 at bottom, -2964 at top

            if (gamepad2.a) {
                handServo1.setPosition(.4);
                handServo2.setPosition(.3);
            }
            if (gamepad2.b) {
                handServo1.setPosition(0);
                handServo2.setPosition(1);
            }

            if (gamepad2.x) {
                handServo1.setPosition(.6);
                handServo2.setPosition(.1);
            }

            if (gamepad2.left_stick_x > 0) {
                relicGrabber.setPosition(1);
            }

            if (gamepad2.left_stick_x < 0) {
                relicGrabber.setPosition(0);
            }


            if (gamepad2.left_bumper) {
                relicMotor.setTargetPosition(RELIC_EXTENDED_POSITION);
                relicMotor.setPower(1);
            }
            if (gamepad2.left_stick_y > 0){
                relicMotor.setTargetPosition(RELIC_EXTENDED_POSITION/3);
                relicMotor.setPower(1);
            }

            if (gamepad2.right_bumper) {
                relicMotor.setTargetPosition(RELIC_CONTRACTED_POSITION);
                relicMotor.setPower(1);
            }

            while (gamepad2.left_stick_button) {
                relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                relicMotor.setPower(gamepad2.left_stick_y / 2);
                if (!gamepad2.left_stick_button) {
                    relicMotor.setPower(0);
                }
            }

            if (gamepad2.left_trigger > 0) {
                extendRelicServo();
            }
            if (gamepad2.right_trigger > 0) {
                contractRelicServo();
            }

            if (gamepad1.right_bumper) {
                speedFactor = 1;
            } else if (gamepad1.left_bumper) {
                speedFactor = .3;
            } else {
                speedFactor = .5;
            }

            motorRight.setPower(gamepad1.left_stick_y * speedFactor);
            motorLeft.setPower(-gamepad1.right_stick_y * speedFactor);

            //telemetry.addData("Right motor encoder Position", motorLeft.getCurrentPosition());
            telemetry.addData("Left motor encoder Position", motorLeft.getCurrentPosition());
            telemetry.update();

        }
    }

    private void extendRelicServo(){
        relicServo2.setPosition(.36);
        sleep(500);
        relicServo1.setPosition(0); //extend
    }

    private void contractRelicServo(){
        relicServo1.setPosition(.85); //contract
        relicServo2.setPosition(1);
    }

}
