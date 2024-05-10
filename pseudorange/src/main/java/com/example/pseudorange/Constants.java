package com.example.pseudorange;

public class Constants {

    public static final long GPS_EPOCH_MILLISECONDS = 315964800000L; // 6 janvier 1980 à 00h00 UTC
    public static final int SECONDS_PER_WEEK = 604800; // 7 jours * 24 heures * 60 minutes * 60 secondes

    public static final double GM = 3.986005e14;
    public static double w_e = 7.2921151467e-5;
}
