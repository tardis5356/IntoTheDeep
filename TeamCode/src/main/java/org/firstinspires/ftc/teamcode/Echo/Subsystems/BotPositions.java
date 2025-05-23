package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .5, GRIPPER_CLOSED = 0.345, GRIPPER_INTAKE = .475;//GRIPPER_CLOSED = 0.34
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .78, WRIST_INTAKE = .4, WRIST_BASKET = .15, WRIST_SPECIMEN = .5;
    public static double ARM_BASKET = .87, ARM_INTAKE = .49, ARM_WALL = .2, ARM_TRANSIT = .5, ARM_SPECIMEN =.68, ARM_SPECIMEN_HANG =.8, ARM_HANG = .61, ARM_START = .7, ARM_SPECIMEN_AUTO =0.66, ARM_SPECIMEN_HANG_AUTO =0.8;
    //ARM_SPECIMEN_AUTO =.65, ARM_SPECIMEN_HANG_AUTO =.76
    public static double INTAKE_ARM_DOWN = .9, INTAKE_ARM_DOWN_AUTO = 0.97,INTAKE_ARM_TRANSFER = 0.38, INTAKE_ARM_UP = .77, INTAKE_ARM_OUTAKE = .16,
            INTAKE_WRIST_DOWN = .85, INTAKE_WRIST_DOWN_AUTO = .95, INTAKE_WRIST_TRANSFER = .05, INTAKE_WRIST_UP = .8, INTAKE_WRIST_OUTAKE = .2,
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0, INTAKE_TRANSFER = 0.4;
    public static double EXTENDO_IN = 0.84, EXTENDO_OUT = .47, EXTENDO_OUT_AUTO = .51, EXTENDO_SPECLEFT =.66, EXTENDO_SPECMID =.54, EXTENDO_SPECRIGHT = .51;

    //These are in motor ticks. On the ITD robot 100 ticks ~= 1 inch. Will differ from robot to robot.
    public static int LIFT_WALL = -2250, LIFT_SPECIMEN_HIGH = -9000, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -49000,
            LIFT_BASKET_LOW = -15500, LIFT_TRANSIT = -14700, LIFT_LIMIT = -51500, LIFT_INTAKE = -1000
            , LIFT_SPECIMEN_HIGH_CLIP = -47250, LIFT_TOLERANCE = 500,LIFT_TOLERANCE_HIGH = 1000, LIFT_TOLERANCE_TIGHT = 500;
    public static double  INTAKE_RED_MIN = 200, INTAKE_BLUE_MIN = 190, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double LIFT_FF = 0.1, WINCH_FF = 0;
    public static double LIFT_P = 0.00016, LIFT_D = 0.000000, LIFT_I = 0.000;
    public static double VINTAKE_TRANSFER = 0.5, VINTAKE_UP = .65, VINTAKE_DOWN = .73, VINTAKE_SWEEP=.83;

    public static int LIFT_WALL_AUTO = -1800, LIFT_SPECIMEN_HIGH_AUTO = -10000,LIFT_BASKET_HIGH_AUTO = -49000, //LIFT_SPECIMEN_HIGH_AUTO = -11000
            LIFT_BASKET_LOW_AUTO = -13350, LIFT_TRANSIT_AUTO = -14700, LIFT_INTAKE_AUTO = -500, LIFT_SPECIMEN_HIGH_CLIP_AUTO = -47250, LIFT_TOLERANCE_AUTO = 500,LIFT_TOLERANCE_TIGHT_AUTO = 250;//1700 is 1 inch

}