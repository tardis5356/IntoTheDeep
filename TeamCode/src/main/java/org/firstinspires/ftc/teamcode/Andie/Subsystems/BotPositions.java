package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .3, GRIPPER_CLOSED = 0.19, GRIPPER_INTAKE = .28;
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .75, WRIST_INTAKE = .67, WRIST_BASKET = .5, WRIST_SPECIMEN = .58;
    public static double ARM_BASKET = .85, ARM_INTAKE = .29, ARM_WALL = .08, ARM_TRANSIT = .5, ARM_SPECIMEN =.57;
    public static double INTAKE_UP = 0.11, INTAKE_DOWN = 0.45, INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0;
    public static double EXTENDO_IN = 0.785, EXTENDO_OUT = .4, EXTENDO_SPECLEFT =.65, EXTENDO_SPECMID =.53, EXTENDO_SPECRIGHT = .4;
    public static int LIFT_WALL = -235, LIFT_SPECIMEN_HIGH = -720, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -2990, LIFT_BASKET_LOW = -280, LIFT_TRANSIT = -500, LIFT_LIMIT = -3000, LIFT_TOLERANCE = 10;
    public static double RED_MAX = 750, RED_MIN = 300, BLUE_MAX = 750, BLUE_MIN = 300, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double ANTI_GRAV = 0.1, WHINCH_FF = 0;
    public static double LIFT_P = 0.009, LIFT_D = 0.0003;
}
