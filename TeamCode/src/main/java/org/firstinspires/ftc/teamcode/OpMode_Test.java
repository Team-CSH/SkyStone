package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@TeleOp(name = "StateMachineTeleOp")
public class OpMode_Test extends OpMode {
    //creez un enumerator pentru states;

    /*
        @param enum - special class that represent a group of constants
     */
    private enum States {
        INIT_STATE,
        LIFT_UP_STATE,
        LIFT_DOWN_STATE,
        SUCTION_INTAKE_STATE,
        SUCTION_OUTTAKE_STATE,
        ARM_EXTENDED_STATE,
        ARM_RETRACTED_STATE,
        PLIERS_ON_STATE,
        PLIERS_OFF_STATE,
        STOP_STATE,
    }

    public DcMotor scissor_lift_left_motor;
    public DcMotor scissor_lift_right_motor;
    public CRServo suction_servo1;
    public CRServo suction_servo2;
    public CRServo expansion_servo;
    public Servo pliers1_servo;
    public Servo pliers2_servo;
    private States CurrentState;
    final double PLIERS_ON = 1;
    final double PLIERS_OFF = 0;

    public boolean resetHardware() {
        scissor_lift_right_motor.setPower(0);
        scissor_lift_left_motor.setPower(0);
        suction_servo1.setPower(0);
        suction_servo2.setPower(0);
        expansion_servo.setPower(0);
        pliers1_servo.setPosition(PLIERS_OFF);
        pliers2_servo.setPosition(PLIERS_OFF);
        return true;
    }

    public void changeState(States p) {
        CurrentState = p;
    }

    public void scissor_lift_motors(double power) {
        scissor_lift_right_motor.setPower(power);
        scissor_lift_left_motor.setPower(power);
    }

    @Override
    public void init() {
        //MOTORS AND HARDWARE MAPS
        scissor_lift_left_motor = hardwareMap.dcMotor.get("scissor_lift_left_motor");
        scissor_lift_right_motor = hardwareMap.dcMotor.get("scissor_lift_right_motor");
        scissor_lift_right_motor.setDirection(DcMotor.Direction.REVERSE);
        //SERVOS
        suction_servo1 = hardwareMap.crservo.get("suction_servo1");
        suction_servo2 = hardwareMap.crservo.get("suction_servo2");
        expansion_servo = hardwareMap.crservo.get("expansion_servo");
        pliers1_servo = hardwareMap.servo.get("pliers1_servo");
        pliers2_servo = hardwareMap.servo.get("pliers2_servo");
    }

    @Override
    public void start() {
        changeState(States.INIT_STATE);
    }

    @Override
    public void loop() {
        switch(CurrentState) {
            case INIT_STATE:
                if(resetHardware()) {
                    sleep(20);
                } else {
                    resetHardware();
                }
                changeState(States.LIFT_UP_STATE);
                break;

            case LIFT_UP_STATE:
                if(gamepad1.left_stick_y > 0) {
                    scissor_lift_motors(1);
                } else {
                    scissor_lift_motors(0);
                }
                changeState(States.LIFT_DOWN_STATE);
                break;

            case LIFT_DOWN_STATE:
                if(gamepad1.left_stick_y < 0) {
                    scissor_lift_motors(-1);
                } else {
                    scissor_lift_motors(0);
                }




        }
    }



}
