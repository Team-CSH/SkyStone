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
        NONE,
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
    private States PreviousState;
    final double PLIERS_ON = 1.0;
    final double PLIERS_OFF = 0.0;

    public boolean resetHardware() { //chestie fara sens doar ca sa dea true
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

    public void changePrevState(States a) {
        PreviousState = a;
    }

    public void scissor_lift_motors(double power) {
        scissor_lift_right_motor.setPower(power);
        scissor_lift_left_motor.setPower(-power);
    }

    public void suction_servos(double power) {
        suction_servo1.setPower(power);
        suction_servo2.setPower(-power);
    }

    public void Arm_Servos(double power) {
        expansion_servo.setPower(power);
    }

    public void pliers(double pos) {
        pliers1_servo.setPosition(pos);
        pliers2_servo.setPosition(-pos);
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
    public void init_loop() {
        // ceva //
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
                    sleep(200);
                }
                changeState(States.LIFT_DOWN_STATE);
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
                changeState(States.SUCTION_INTAKE_STATE);
                break;

            case SUCTION_INTAKE_STATE:
                if(gamepad1.a) {
                    suction_servos(1);
                } else {
                    suction_servos(0);
                }
                changeState(States.SUCTION_OUTTAKE_STATE);
                break;

            case SUCTION_OUTTAKE_STATE:
                if(gamepad1.y) {
                    suction_servos(-1.0);
                } else {
                    suction_servos(0.0);
                }
                changeState(States.ARM_EXTENDED_STATE);
                break;

            case ARM_EXTENDED_STATE:
                if(gamepad1.right_bumper) {
                    Arm_Servos(1);
                } else {
                    Arm_Servos(0);
                }
                changeState(States.ARM_RETRACTED_STATE);
                break;

            case ARM_RETRACTED_STATE:
                if(gamepad1.left_bumper) {
                    Arm_Servos(-1);
                } else {
                    Arm_Servos(0);
                }
                changeState(States.PLIERS_ON_STATE);
                break;

            case PLIERS_ON_STATE:
                if(gamepad1.x) {
                    pliers(PLIERS_ON);
                }
                changeState(States.PLIERS_OFF_STATE);
                break;

            case PLIERS_OFF_STATE:
                if(gamepad1.b) {
                    pliers(PLIERS_OFF);
                }
                changeState(States.STOP_STATE);
                break;

            case STOP_STATE:
                resetHardware();
                changeState(States.INIT_STATE);
                break;
        }
    }
}
