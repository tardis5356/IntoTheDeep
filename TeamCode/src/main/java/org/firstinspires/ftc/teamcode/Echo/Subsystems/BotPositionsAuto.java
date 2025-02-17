package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositionsAuto {
    public static double GRIPPER_OPEN = .6, GRIPPER_CLOSED = 0.338, GRIPPER_INTAKE = .475;
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .78, WRIST_INTAKE = .37, WRIST_BASKET = .15, WRIST_SPECIMEN = .5;
    public static double ARM_BASKET = .85, ARM_INTAKE = .44, ARM_WALL = .2, ARM_TRANSIT = .5, ARM_SPECIMEN =.7, ARM_SPECIMEN_HANG =.78, ARM_HANG = .61, ARM_START = .7;
    public static double INTAKE_ARM_DOWN = .9, INTAKE_ARM_DOWN_AUTO = 0.97,INTAKE_ARM_TRANSFER = 0.38, INTAKE_ARM_UP = .77, INTAKE_ARM_OUTAKE = .16,
            INTAKE_WRIST_DOWN = .85, INTAKE_WRIST_DOWN_AUTO = .95, INTAKE_WRIST_TRANSFER = .05, INTAKE_WRIST_UP = .8, INTAKE_WRIST_OUTAKE = .2,
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0, INTAKE_TRANSFER = 0.4;
    public static double EXTENDO_IN = 0.8, EXTENDO_OUT = .5, EXTENDO_SPECLEFT =.65, EXTENDO_SPECMID =.53, EXTENDO_SPECRIGHT = .5;

    //These are in motor ticks. On the ITD robot 100 ticks ~= 1 inch. Will differ from robot to robot.
    public static int LIFT_WALL = -1800, LIFT_SPECIMEN_HIGH = -9000, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -49000,
            LIFT_BASKET_LOW = -13350, LIFT_TRANSIT = -14700, LIFT_LIMIT = -50250, LIFT_INTAKE = -7700, LIFT_SPECIMEN_HIGH_CLIP = -47250, LIFT_TOLERANCE = 1500,LIFT_TOLERANCE_MED = 35, LIFT_TOLERANCE_TIGHT = 800;//100 is 1 inch
    public static double  INTAKE_RED_MIN = 200, INTAKE_BLUE_MIN = 190, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double LIFT_FF = 0.1, WINCH_FF = 0;
    public static double LIFT_P = 0.0002, LIFT_D = 0.0000, LIFT_I = 0.000;
    public static double VINTAKE_TRANSFER = 0.45, VINTAKE_UP = .7, VINTAKE_DOWN = .8, VINTAKE_SWEEP=.9;

}
