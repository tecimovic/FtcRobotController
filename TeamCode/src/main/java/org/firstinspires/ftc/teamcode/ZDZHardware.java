package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ZDZHardware {
    // Declare OpMode members.
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final DcMotorEx launcher;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;

    ZDZHardware(HardwareMap hardwareMap) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
    }

    public CRServo rightFeeder() { return rightFeeder; }
    public CRServo leftFeeder() { return leftFeeder; }
    public DcMotor leftDrive() { return leftDrive; }
    public DcMotor rightDrive() { return rightDrive; }
    public DcMotorEx launcher() { return launcher; }
}
