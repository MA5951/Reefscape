package com.ma5951.utils.Logger;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private int autoLogNum = 0;
    private int teleopLogNum = 0;
    private int testLogNum = 0;
    private boolean compitionLog;
    private boolean wasTeleop = false;
    private static final ZoneId m_utc = ZoneId.of("UTC");
    private static final DateTimeFormatter m_timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
            .withZone(m_utc);
    private LocalDateTime now;

    public MALog(boolean competitionStyleLog) {
        compitionLog = competitionStyleLog;
        DataLogManager.logNetworkTables(true);

        if (competitionStyleLog) {
            startLog("");
        }
    }

    private void startLog(String logName) {
        DataLogManager.start("", logName);
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    public void stopLog() {
        if (!compitionLog) {
            DataLogManager.stop();
        } else {
            if (wasTeleop) {
                DataLogManager.stop();
            }
        }
    }

    public void startAutoLog() {
        if (!compitionLog) {
            now = LocalDateTime.now(m_utc);
            startLog("AutoLog_A" + autoLogNum + "_" + m_timeFormatter.format(now)+ ".wpilog");
            autoLogNum++;
        }
    }

    public void startTeleopLog() {
        if (!compitionLog) {
            now = LocalDateTime.now(m_utc);
            startLog("TeleLog_T" + teleopLogNum + "_" + m_timeFormatter.format(now)+ ".wpilog");
            teleopLogNum++;
        } else {
            wasTeleop = true;
        }
    }

    public void startTestLog() {
        if (!compitionLog) {
            now = LocalDateTime.now(m_utc);
            startLog("TestLog_TE" + testLogNum + "_" + m_timeFormatter.format(now)+ ".wpilog");
            testLogNum++;
        }
    }

    public static MALog getInstance(boolean isCompetition) {
        if (logger == null) {
            logger = new MALog(isCompetition);
        }
        return logger;
    }

}
