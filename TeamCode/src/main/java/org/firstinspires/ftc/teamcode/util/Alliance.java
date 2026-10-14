package org.firstinspires.ftc.teamcode.util;

/** Which side we're on. Picked on the Driver Hub during INIT. */
public enum Alliance {
    RED("RED"), BLUE("BLUE");

    /** Start of the HIVE CELL cluster names that belong to this alliance. */
    public final String cellPrefix;

    Alliance(String cellPrefix) {
        this.cellPrefix = cellPrefix;
    }
}
