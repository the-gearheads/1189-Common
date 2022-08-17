package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;

public class Controllers {
    private static String[] lastControllerNames = new String[6];

    public static SingleXboxController driverController = new SingleXboxController(0);
    public static OperatorController operatorController = new OperatorController(1);

    public enum ControllerMode {
        NORMAL, TESTING
    }
    public static ControllerMode mode = ControllerMode.NORMAL;

    /** Returns true if the connected controllers have changed since last called. */
    public static boolean didControllersChange() {
        boolean hasChanged = false;

        for (int i=0; i<DriverStation.kJoystickPorts; i++) {
            String name = DriverStation.getJoystickName(i);
            if(!name.equals(lastControllerNames[i])) {
                hasChanged = true;
                lastControllerNames[i] = name;
            }
        }

        return hasChanged;
    }

    public static void updateActiveControllerInstance() {
        /* Just find the first Xbox controller */
        int controllerCounter = 0;
        for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
            if (!DriverStation.getJoystickName(i).equals("")) {
                if(controllerCounter == 1)
                    driverController = new SingleXboxController(i);
                else if(controllerCounter == 2){
                    operatorController = new OperatorController(i);
                    return;
                }
                controllerCounter++;
            }
        }
        driverController = new SingleXboxController(0); // No controller found, but a NullPointerException would be far worse than any warnings
        operatorController = new OperatorController(1); // No controller found, but a NullPointerException would be far worse than any warnings
    }
}
