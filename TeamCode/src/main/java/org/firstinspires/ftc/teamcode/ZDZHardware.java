package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ZDZHardware {
    // Declare OpMode members.
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final DcMotorEx launcher;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;
    private final Servo light;

    private final DistanceSensor frontDistance;
    private final NormalizedColorSensor colorSensor;
    private final ElapsedTime timer;

    private final LED ledLeftRed;
    private final LED ledLeftGreen;
    private final LED ledRightRed;
    private final LED ledRightGreen;

    private int tickCount = 0;


    ZDZHardware(HardwareMap hardwareMap) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        light = hardwareMap.get(Servo.class, "light");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        ledLeftRed = hardwareMap.get(LED.class, "led_left_red");
        ledLeftGreen = hardwareMap.get(LED.class, "led_left_green");
        ledRightRed = hardwareMap.get(LED.class, "led_right_red");
        ledRightGreen = hardwareMap.get(LED.class, "led_right_green");

        lightWhite();
        timer = new ElapsedTime();
        timer.reset();
    }


    public NormalizedRGBA colorSensor() {
        return colorSensor.getNormalizedColors();
    }

    public double frontDistanceInCentimeters() {
        return frontDistance.getDistance(DistanceUnit.CM);
    }

    public void initFeeders() {
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reverseDriveDirection() {
        leftDrive.setDirection(leftDrive.getDirection().inverted());
        rightDrive.setDirection(rightDrive.getDirection().inverted());

    }

    public void setDrivePower(double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    public int getLeftPosition() {
        return leftDrive.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightDrive.getCurrentPosition();
    }

    public void setTargetPositions(int leftTargetPosition, int rightTargetPosition) {
        leftDrive.setTargetPosition(leftTargetPosition);
        rightDrive.setTargetPosition(rightTargetPosition);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int targetPosition) {
        leftDrive.setTargetPosition(targetPosition);
        rightDrive.setTargetPosition(targetPosition);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void simpleDrive(double speed) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    public void initDrive(boolean isAuto) {

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make the robot go forward. So, adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90° drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        if ( isAuto ) {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

    }

    public void initLauncher() {
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public void setLauncherVelocity(double v) {
        launcher.setVelocity(v);
    }

    public void lightwhee() {
        for (double x = 0.280; x >= 0.723; x += 0.01) {
            light.setPosition(x);
            halfSecondTick(1);

        }
        for (double x = 0.723; x <= 0.280; x -= 0.01) {
            light.setPosition(x);
            halfSecondTick(1);

        }

    }

    public void lightWhite() {
        light.setPosition(1.0);
    }

    public void lightGreen() {
        light.setPosition(0.5);
    }

    public void lightOff() {
        light.setPosition(0.0);
    }

    public void lightRed() {
        light.setPosition(0.280);
    }

    public void lightBlue() {
        light.setPosition(0.63);
    }

    public void ledsOff() {
        this.ledRightRed.off();
        this.ledLeftRed.off();
        this.ledRightGreen.off();
        this.ledLeftGreen.off();
    }

    public void ledsRed() {
        this.ledRightRed.on();
        this.ledLeftRed.on();
        this.ledRightGreen.off();
        this.ledLeftGreen.off();
    }

    public void ledsGreen() {
        this.ledRightRed.off();
        this.ledLeftRed.off();
        this.ledRightGreen.on();
        this.ledLeftGreen.on();
    }

    public void stopAll() {
        leftDrive.setPower(0.0);
        leftFeeder.setPower(0.0);
        rightDrive.setPower(0.0);
        rightFeeder.setPower(0.0);
        launcher.setPower(0.0);
    }

    public void init_loop() {
        if (timer.milliseconds() > 500) {
            timer.reset();
            halfSecondTick(tickCount++);
        }
    }

    public void drivePower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void feederPower(double pwr) {
        leftFeeder.setPower(pwr);
        rightFeeder.setPower(pwr);
    }


    public void showState(Telemetry t) {
        t.addData("Front distance (cm)", frontDistanceInCentimeters());
        t.addData("Colors", Math.round(colorSensor().red * 10000) + "/" + Math.round(colorSensor().green * 10000) + "/" + Math.round(colorSensor().blue * 10000));
        t.addData("Motors current", "left (%d), right (%d)", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        t.addData("Motors target", "left (%d), right (%d)", leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
        t.addData("Launcher", "velocity (%.6f)", launcher.getVelocity());
        t.update();
    }

    private void halfSecondTick(int count) {
        switch (count % 3) {
            case 0:
                lightRed();
                ledsRed();
                break;
            case 1:
                lightGreen();
                ledsGreen();
                break;
            case 2:
                lightBlue();
                ledsOff();
                break;
        }
    }
}