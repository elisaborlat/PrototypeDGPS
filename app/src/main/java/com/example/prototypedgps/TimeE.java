package com.example.prototypedgps;

import java.util.Locale;

public class TimeE {

    private static final long MILLISECONDS_IN_WEEK = 604800000L; // 7 jours en millisecondes
    //private static final long GPS_EPOCH_MILLISECONDS = 315964800000L; // 6 janvier 1980 en millisecondes

    private final double gpsTimeMilliseconds; // Temps GPS en millisecondes depuis le début du temps GPS
    private final Cal gpsTimeCalendar;
    private final FormatTime gps;

    private final FormatTime mjd;

    public TimeE(Cal gpsTimeCalendar){
        this.gpsTimeCalendar = gpsTimeCalendar;
        this.gps = cal2gps(gpsTimeCalendar);
        this.mjd = cal2mjd(gpsTimeCalendar);
        this.gpsTimeMilliseconds =  gps.integerPart*MILLISECONDS_IN_WEEK + gps.doublePart*1000;
    }

    public TimeE(FormatTime gps) {
        this.gpsTimeCalendar = gps2cal(gps);
        this.gps = gps;
        this.gpsTimeMilliseconds = gps.integerPart*MILLISECONDS_IN_WEEK + gps.doublePart*1000;
        this.mjd = gps2mjd(gps);
    }

    public TimeE(double gpsTimeMilliseconds) {
        this.gpsTimeMilliseconds = gpsTimeMilliseconds;
        this.gps = gpsTimeMilliseconds2gps(gpsTimeMilliseconds);
        this.gpsTimeCalendar = gps2cal(gps);
        this.mjd = gps2mjd(gps);
    }

    public static TimeE fromMjd(FormatTime mjd) {
        FormatTime gps = mjd2gps(mjd);
        return new TimeE(gps);
    }

    public Cal gps2cal(FormatTime gps) {

        FormatTime jd = gps2jd(gps);

        return jd2cal(jd);
    }

    public FormatTime gps2jd(FormatTime gps2) {

        Cal cal0 = new Cal(1980, 1, 6, 0, 0, 0.0);
        FormatTime jd0 = cal2jd(cal0);

        double jdDayTemp = jd0.integerPart + jd0.doublePart / 24.0 / 3600.0 + 7.0 * gps2.integerPart + gps2.doublePart / 24.0 / 3600.0;
        int jdDay = (int) jdDayTemp;

        double jdSecOfDay = (jdDayTemp - jdDay) * 24.0 * 3600.0;
        return new FormatTime(jdDay, jdSecOfDay);
    }

    public double getGpsTimeMilliseconds() {
        return gpsTimeMilliseconds;
    }


    public Cal getGpsTimeCalendar() {
        return gpsTimeCalendar;
    }

    public int getGpsWeek() {
        return gps.integerPart;
    }

    public double getTow() {
        return gps.doublePart;
    }


    public String toString() {
        return "Temps GPS: " +
                "\n- Millisecondes depuis le début du temps GPS: " + gpsTimeMilliseconds +
                "\n- Temps calendaire (secondes): " + gpsTimeCalendar +
                "\n- Semaine GPS: " + getGpsWeek() + "| TOW: " + getTow() ;
    }

    public static FormatTime cal2jd(Cal cal) {

        int year = cal.getYear();
        int month = cal.getMonth();
        int day = cal.getDay();
        int hour = cal.getHour();
        int minute = cal.getMinutes();
        double second = cal.getSec();

        int Y;
        int M;

        if (month > 2) {
            Y = year;
            M = month;
        } else {
            Y = year - 1;
            M = month + 12;
        }

        double D = day + hour / 24.0 + minute / (24.0 * 60.0) + second / (24.0 * 3600.0);
        int B = 2 - (int) Math.floor(Y / 100) + (int) Math.floor(Y / 400);

        // Calculate Julian Day
        double jd = Math.floor(365.25 * (Y + 4716)) + Math.floor(30.6001 * (M + 1)) + D + B - 1524.5;

        // Calculate fractional part of the day
        double jdDay = hour / 24.0 + minute / (24.0 * 60.0) + second / (24.0 * 3600.0);

        // Calculate Julian Day seconds based on the fractional part
        double jdSecOfDay;
        if (jdDay < 0.5) {
            jdSecOfDay = 12 * 3600.0 + hour * 3600.0 + minute * 60.0 + second;
        } else {
            jdSecOfDay = -12 * 3600.0 + hour * 3600.0 + minute * 60.0 + second;
        }

        return new FormatTime((int) Math.floor(jd), jdSecOfDay);
    }

    public FormatTime cal2gps(Cal cal) {

        FormatTime mjd = cal2mjd(cal);

        return mjd2gps(mjd);
    }

    public static FormatTime mjd2gps(FormatTime mjd) {
        int mjdDay = mjd.getIntegerPart();

        double mjdSecOfDay = mjd.getSec();

        int jdDay = (int) Math.floor(2400000.5 + mjdDay + mjdSecOfDay / (24.0 * 3600.0));

        double gpsSecOfDay = mjdSecOfDay + 0.5 * 24.0 * 3600.0;
        if (gpsSecOfDay > 24.0 * 3600.0) {
            gpsSecOfDay -= 24.0 * 3600.0;
        }

        return jd2gps(new FormatTime(jdDay, gpsSecOfDay));
    }

    public static FormatTime jd2gps(FormatTime jd) {
        // Calcul du temps GPS = 0 (6 janvier 1980) en date julienne
        Cal cal0 = new Cal(1980, 1, 6, 0, 0, 0.0);
        FormatTime jd0 = cal2jd(cal0);
        double djd = jd.integerPart - jd0.integerPart + (jd.doublePart - jd0.doublePart) / (24.0 * 3600.0);
        // Calcul de la semaine GPS
        int week = (int) Math.floor(djd / 7);
        // Calcul de la seconde de semaine GPS
        double sec = (djd - week * 7) * 24.0 * 3600.0;
        return new FormatTime(week, sec);
    }


    public FormatTime cal2mjd(Cal cal) {

        // Calcul de la date julienne
        FormatTime jd = cal2jd(cal);

        // Calcul du jour en date julienne modifiée
        int mjdDay = (int) Math.floor((jd.integerPart + jd.doublePart / (24.0 * 3600.0)) - 2400000.5);

        // Calcul de la seconde de jour en date julienne modifiée
        double mjdSecOfDay = jd.doublePart - 12.0 * 3600.0;
        if (mjdSecOfDay < 0) {
            mjdSecOfDay += 24.0 * 3600.0;
        }

        return new FormatTime(mjdDay, mjdSecOfDay);
    }
    public Cal mjd2cal(FormatTime mjd) {

        int jdDay = (int) Math.floor(mjd.integerPart+ mjd.doublePart / (24 * 3600) + 2400000.5);

        double jdSecOfDay2 = mjd.doublePart + 12.0 * 3600.0;
        if (jdSecOfDay2 > 24.0 * 3600.0) {
            jdSecOfDay2 -= 24.0 * 3600.0;
        }

        return jd2cal(new FormatTime(jdDay,jdSecOfDay2));
    }
    public Cal jd2cal(FormatTime jdTime) {

        int jdDay = jdTime.integerPart;
        double jdSecOfDay = jdTime.doublePart;

        // Combine Julian Day and fractional part
        double djd = jdDay + jdSecOfDay / (24.0 * 3600.0) + 0.5;

        // Extract whole Julian Day
        int jd = (int) Math.floor(djd);

        // Calculate fractional part representing hours
        double fhour = (jdSecOfDay / (24.0 * 3600.0) + 0.5) % 1.0 * 24.0;

        // Extract hour
        int hour = (int) Math.floor(fhour);

        // Calculate minute
        int minute = (int) Math.floor((fhour - hour) * 60.0);

        // Calculate second
        double second = (fhour - (int) fhour - (double) minute / 60.0) * 3600.0;

        // Algorithmic conversion to Gregorian calendar
        int k = jd + 68569;
        int n = 4 * k / 146097;
        k = k - (146097 * n + 3) / 4;
        int m = 4000 * (k + 1) / 1461001;
        k = k - (1461 * m) / 4 + 31;

        int month = 80 * k / 2447;
        int day = k - 2447 * month / 80;
        k = month / 11;

        month = month + 2 - 12 * k;
        int year = 100 * (n - 49) + m + k;

        return new Cal(year, month, day, hour, minute, second);
    }

    public FormatTime gpsTimeMilliseconds2gps(double gpsTimeMilliseconds){
        long week = (long) gpsTimeMilliseconds / MILLISECONDS_IN_WEEK ;
        int weekint = (int) week;
        double sec = (gpsTimeMilliseconds - week*MILLISECONDS_IN_WEEK)/1000;

        return new FormatTime(weekint, sec);
    }

    private FormatTime gps2mjd(FormatTime gps){
        Cal temp = gps2cal(gps);
        FormatTime temp2 = cal2mjd(temp);

        int mjdDay = temp2.integerPart;
        double mjdSecOfDay = temp2.doublePart;

        return new FormatTime(mjdDay, mjdSecOfDay);
    }


    public FormatTime getGps() {
        return gps;
    }

    public FormatTime getMjd() {
        return mjd;
    }


    public static class FormatTime {

        public int integerPart;
        public double doublePart;

        public FormatTime(int integerPart, double doublePart) {
            this.integerPart = integerPart;
            this.doublePart = doublePart;
        }

        public int getIntegerPart() {
            return integerPart;
        }

        public double getSec() {
            return doublePart;
        }

    }

    public static class Cal {

        private final int year;
        private final int month;
        private final int day;
        private final int hour;
        private final int minutes;
        private final double sec;

        public Cal(int year, int month, int day, int hour, int minutes, double sec) {
            this.year = year;
            this.month = month;
            this.day = day;
            this.hour = hour;
            this.minutes = minutes;
            this.sec = sec;

        }

        public double getSec() {
            return sec;
        }

        public int getHour() {
            return hour;
        }

        public int getMinutes() {
            return minutes;
        }

        public int getDay() {
            return day;
        }

        public int getYear() {
            return year;
        }
        public int getMonth() {
            return month;
        }

        public String toString(){

            return String.format(Locale.US,"%02d-%02d-%d %02d:%02d:%-6.3f", day, month, year, hour, minutes, sec);
        }

    }


}