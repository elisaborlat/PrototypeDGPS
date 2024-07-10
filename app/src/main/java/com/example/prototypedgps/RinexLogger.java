package com.example.prototypedgps;

import android.content.Context;
import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.location.Location;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.Month;
import java.util.Date;
import java.util.Locale;

public class RinexLogger implements MeasurementListener {

    private static final String TAG = "RinexLogger";
    private static final String FILE_PREFIX = "rinex";
    private static final String ERROR_WRITING_FILE = "Problem writing to file.";

    private final Context mContext;

    private final Object mFileLock = new Object();
    private BufferedWriter mFileWriter;
    private File mFile;

    private boolean firstMes = true;

    private static final int TOW_DECODED_MEASUREMENT_STATE_BIT = 3;
    private static final int C_TO_N0_THRESHOLD_DB_HZ = 18;
    Boolean set_clockBias = false;
    private static double fullBiasNanos = 1.0e-9,BiasNanos = 1.0e-9;

    private HomeFragment.HomeUIFragmentComponent mUiFragmentComponent;


    public synchronized void setUiFragmentComponent(HomeFragment.HomeUIFragmentComponent value) {
        mUiFragmentComponent = value;
    }

    public RinexLogger(Context context) {
        this.mContext = context;
    }

    /** Start a new file logging process.
     * Source : GNSSLogger */
    public void startNewLog() {
        synchronized (mFileLock) {

            firstMes=true;
            mUiFragmentComponent.resetRinexCounter();

            String state = Environment.getExternalStorageState();
            if (!Environment.MEDIA_MOUNTED.equals(state)) {
                if (Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
                    logError("Cannot write to external storage.");
                } else {
                    logError("Cannot read external storage.");
                }
                return;
            }

            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss",Locale.US);
            Date now = new Date();
            String fileName = String.format("%s_%s.txt", FILE_PREFIX, formatter.format(now));
            File currentFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), fileName);
            String currentFilePath = currentFile.getAbsolutePath();
            BufferedWriter currentFileWriter;
            try {
                currentFileWriter = new BufferedWriter(new FileWriter(currentFile));
            } catch (IOException e) {
                logException("Could not open file: " + currentFilePath, e);
                return;
            }

            if (mFileWriter != null) {
                try {
                    mFileWriter.close();
                } catch (IOException e) {
                    logException("Unable to close all file streams.", e);
                    return;
                }
            }

            mFile = currentFile;
            mFileWriter = currentFileWriter;
            Toast.makeText(mContext, "File opened: " + currentFilePath, Toast.LENGTH_SHORT).show();

        }
    }


    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {

    }

    @Override
    public void onLocationChanged(Location location) {

    }

    @Override
    public void onLocationStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {

        GnssClock gnssClock = event.getClock();
        // Check that the receiver have estimate GPS time
        if (!gnssClock.hasFullBiasNanos()) {
            System.out.println("RinexLogger| FullBiasNanos is not decoded, exit log meas");
            return;
        }

        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
                try {
                    String measurementStream = gnssMeasurementsEventToString(event);
                    mFileWriter.write(measurementStream);

                    if (mUiFragmentComponent != null) {
                        mUiFragmentComponent.incrementRinexCounter();
                    }
                } catch (IOException e) {
                    logException(ERROR_WRITING_FILE, e);
                }
        }
    }

    @Override
    public void onGnssMeasurementsStatusChanged(int status) {

    }

    @Override
    public void onGnssNavigationMessageReceived(GnssNavigationMessage event) {

    }

    @Override
    public void onGnssNavigationMessageStatusChanged(int status) {

    }

    @Override
    public void onGnssStatusChanged(GnssStatus gnssStatus) {

    }

    @Override
    public void onListenerRegistration(String listener, boolean result) {

    }

    @Override
    public void onNmeaReceived(long l, String s) {

    }

    @Override
    public void onTTFFReceived(long l) {

    }

    public void writeRinexHeader(String rinexHeader){
        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            try {
                mFileWriter.write(rinexHeader);
            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }
    }

    private void logException(String errorMessage, Exception e) {
        Log.e(MeasurementProvider.TAG + TAG, errorMessage, e);
        Toast.makeText(mContext, errorMessage, Toast.LENGTH_LONG).show();
    }

    private void logError(String errorMessage) {
        Log.e(MeasurementProvider.TAG + TAG, errorMessage);
        Toast.makeText(mContext, errorMessage, Toast.LENGTH_LONG).show();
    }

    private String gnssMeasurementsEventToString(GnssMeasurementsEvent event) {
        // Data record description (Ref: TABLE A3)
        // Init StringBuilder
        StringBuilder builder = new StringBuilder();

        // EPOCH
        GnssClock gnssClock = event.getClock();

        long GPSTime = gnssClock.getTimeNanos() - gnssClock.getFullBiasNanos();
        LocalDateTime gpsEpoch = LocalDateTime.of(1980, Month.JANUARY, 6, 0, 0, 0);
        LocalDateTime gpsCalendarTime = gpsEpoch.plusNanos(GPSTime);

        // Write header if first mes
        if (firstMes) {
            String header = rinexHeader(gpsCalendarTime);
            writeRinexHeader(header);
            firstMes = false;
        }

        //OBS
        int numberOfMes = 0;
        StringBuilder obsStringBuilder = new StringBuilder();
        for (GnssMeasurement mes : event.getMeasurements()) {

            if (mes.getConstellationType() != GnssStatus.CONSTELLATION_GPS) {
                continue; // interrupt current iteration if not a gps satellite
            }

            if (mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                    && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0) {

                /*maintaining constant the 'FullBiasNanos' instead of using the instantaneous value. This avoids the 256 ns
                 jumps each 3 seconds that create a code-phase divergence due to the clock.*/
                if (!set_clockBias) {
                    fullBiasNanos = gnssClock.getFullBiasNanos();
                    BiasNanos = gnssClock.getBiasNanos();
                    set_clockBias = true;
                }

                double tTx = mes.getReceivedSvTimeNanos();
                double tRxGNSS = gnssClock.getTimeNanos() + mes.getTimeOffsetNanos() - (fullBiasNanos + BiasNanos);
                double tRx = tRxGNSS % Constants.NUMBER_NANO_SECONDS_WEEK;
                double pseudorange = (tRx - tTx) * 1e-9 * Constants.SPEED_OF_LIGHT;

                int satId = mes.getSvid();
                String constellationId = getConstellationSystemIdentifierRinex(mes.getConstellationType());

                double carrierToNoiseRatio = mes.getCn0DbHz();
                double doppler = -mes.getPseudorangeRateMetersPerSecond() / (Constants.SPEED_OF_LIGHT / mes.getCarrierFrequencyHz());
                double carrier = mes.getAccumulatedDeltaRangeMeters() / (Constants.SPEED_OF_LIGHT / mes.getCarrierFrequencyHz());

                obsStringBuilder.append(String.format(Locale.US, "%1s%02d%14.3f  %14.3f  %14.3f  %14.3f%n",
                        constellationId,
                        satId,
                        pseudorange,
                        carrier,
                        doppler,
                        carrierToNoiseRatio));
                numberOfMes = numberOfMes + 1;
            }
        }

        // Append EPOCH
        double seconds = (double) gpsCalendarTime.getSecond();
        double nanos = gpsCalendarTime.getNano() / 1e9;
        double secondRinex = seconds + nanos;
        builder.append(String.format(Locale.US, "> %04d %02d %02d %02d %02d %10.7f %2d %2d%n",
                gpsCalendarTime.getYear(),
                gpsCalendarTime.getMonthValue(),
                gpsCalendarTime.getDayOfMonth(),
                gpsCalendarTime.getHour(),
                gpsCalendarTime.getMinute(),
                secondRinex, 0, numberOfMes));
        // Append OBS
        builder.append(obsStringBuilder);

        return builder.toString();
    }

    private String rinexHeader(LocalDateTime startTime) {

        // Date info from the smartphone
        StringBuilder builder = new StringBuilder();

        int year = startTime.getYear();
        int month = startTime.getMonthValue();
        int day = startTime.getDayOfMonth();
        int hour = startTime.getHour();
        int minute = startTime.getMinute();
        int second = startTime.getSecond();
        double secondDouble = (double) startTime.getSecond();
        double nanos = startTime.getNano() / 1e9;
        double secondRinex = secondDouble + nanos;

        String dateRinex = String.format(Locale.US,"%04d%02d%02d", year, month, day);
        String timeRinex = String.format(Locale.US,"%02d%02d%02d", hour, minute, second - 18);

        // Header
        String[] header = {"3.03", "", "OBSERVATION DATA", "G: GPS", "RINEX VERSION / TYPE"};
        String[] pgmRunByDate = {"BORLAT", "", dateRinex + " " + timeRinex + " UTC", "PGM / RUN BY / DATE"};
        String[] markerName = {"Unknown", "MARKER NAME"};
        String[] markerNumber = {"Unknown", "MARKER TYPE"};
        String[] observerAgency = {"", "OBSERVER / AGENCY"};
        String[] recTypeVers = {"Unknown", "PrototypeDGPS", "vx.x.x", "REC # / TYPE / VERS"};
        String[] antType = {"", "Unknown", "NONE", "ANT # / TYPE"};
        String[] approxPositionAntenna = {"Unknown", "Unknown", "", "", "APPROX POSITION XYZ"};
        String[] antennaDeltas = {"0.0000", "0.0000", "0.0000", "", "ANTENNA: DELTA H/E/N"};
        String[] observationType = {"G", "", "4", "C1C L1C D1C S1C", "SYS / # / OBS TYPES"};

        builder.append(String.format("%9s%11s%-20s%-20s%-20s%n", header[0], header[1], header[2], header[3], header[4]));
        builder.append(String.format("%-20s%-20s%-20s%-20s%n", pgmRunByDate[0], pgmRunByDate[1], pgmRunByDate[2], pgmRunByDate[3]));
        builder.append(String.format("%-60s%-20s%n", markerName[0], markerName[1]));
        builder.append(String.format("%-60s%-20s%n", markerNumber[0], markerNumber[1]));
        builder.append(String.format("%-60s%-20s%n", observerAgency[0], observerAgency[1]));
        builder.append(String.format("%-20s%-20s%-20s%-20s%n", recTypeVers[0], recTypeVers[1], recTypeVers[2], recTypeVers[3]));
        builder.append(String.format("%-20s%-16s%-24s%-20s%n", antType[0], antType[1], antType[2], antType[3]));
        builder.append(String.format("%14s%14s%14s%-18s%-20s%n", approxPositionAntenna[0], approxPositionAntenna[1], approxPositionAntenna[2], approxPositionAntenna[3], approxPositionAntenna[4]));
        builder.append(String.format("%14s%14s%14s%-18s%-20s%n", antennaDeltas[0], antennaDeltas[1], antennaDeltas[2], antennaDeltas[3], antennaDeltas[4]));
        builder.append(String.format("%1s%2s%3s%-54s%-20s%n", observationType[0], observationType[1], observationType[2], observationType[3], observationType[4]));

        builder.append(String.format(Locale.US, "%6d    %02d    %02d    %02d    %02d%13.7f     %3s         %-20s%n",
                year,
                month,
                day,
                hour,
                startTime.getMinute(),
                secondRinex,
                "GPS",
                "TIME OF FIRST OBS"));

        builder.append(String.format("%1s %3s%55s%-20s%n","G","L1C","","SYS / PHASE SHIFT"));

        builder.append(String.format(" 0%58s%-20s%n", "","GLONASS SLOT / FRQ #"));

        builder.append(" C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000        GLONASS COD/PHS/BIS\n");
        builder.append("                                                            END OF HEADER\n");

        return builder.toString();
    }


    public void send() {
        if (mFile == null) {
            return;
        }
        if (mFileWriter != null) {
            try {
                mFileWriter.flush();
                mFileWriter.close();
                mFileWriter = null;
            } catch (IOException e) {
                logException("Unable to close all file streams.", e);
            }
        }
    }

    private String getConstellationSystemIdentifierRinex(int id) {
        switch (id) {
            case 1:
                return "G";
            case 2:
                return "S";
            case 3:
                return "R";
            case 4:
                return "J";
            case 5:
                return "C";
            case 6:
                return "E";
            default:
                return "UNKNOWN";
        }
    }
}
