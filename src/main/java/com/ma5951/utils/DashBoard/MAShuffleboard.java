package com.ma5951.utils.DashBoard;

import java.util.HashMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MAShuffleboard {
    private ShuffleboardTab board;
    private HashMap<String, GenericEntry> values;
    

    public MAShuffleboard(String tab) {
        board = Shuffleboard.getTab(tab);
        values = new HashMap<>();
        
        
    }

    public void addNum(String title, double num) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, num).getEntry());
        } else {
            values.get(title).setDouble(num);
        }
    }

    public void addString(String title, String str) {
        if(!values.containsKey(title)) {
            values.put(title, board.add(title, str).getEntry());
        } else {
            values.get(title).setString(str);
        }
    }

    public void addBoolean(String title, boolean bol) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, bol).getEntry());
        } else {
            values.get(title).setBoolean(bol);
        }
    }

    public double getNum(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getDouble(0);
        }
        System.err.println("none existing title: " + title);
        return 0;
    }

    public String getString(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getString("null");
        }
        System.err.println("none existing title: " + title);
        return "null";
    }

    public Boolean getBoolean(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getBoolean(false);
        }
        System.err.println("none existing title: " + title);
        return false;
    }

    public pidControllerGainSupplier getPidControllerGainSupplier(String PIDname, double KP, double KI, double KD) {
        return new pidControllerGainSupplier(this, PIDname, KP, KI, KD);
    }

    public pidControllerGainSupplier getPidControllerGainSupplier(String PIDname) {
        return this.getPidControllerGainSupplier(PIDname,0, 0, 0);
    }

    public class pidControllerGainSupplier {
        private String KP_STRING;
        private String KI_STRING;
        private String KD_STRING;
        private MAShuffleboard shuffleboard;

        private pidControllerGainSupplier(MAShuffleboard shuffleboard, String PIDname, double KP, double KI, double KD) {
            this.shuffleboard = shuffleboard;
            KP_STRING = PIDname + " KP";
            KI_STRING = PIDname + " KI";
            KD_STRING = PIDname + " KD";
            shuffleboard.addNum(KP_STRING, KP);
            shuffleboard.addNum(KI_STRING, KI);
            shuffleboard.addNum(KD_STRING, KD);
        }

        public double getKP() {
            return shuffleboard.getNum(KP_STRING);
        }

        public double getKI() {
            return shuffleboard.getNum(KI_STRING);
        }

        public double getKD() {
            return shuffleboard.getNum(KD_STRING);
        }
    }

    public void addCommand(String title , Command Command) {
        SmartDashboard.putData(title, Command);
    }

    
}
