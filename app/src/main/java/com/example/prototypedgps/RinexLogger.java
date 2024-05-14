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

import androidx.fragment.app.FragmentActivity;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileFilter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.time.Month;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class RinexLogger implements MeasurementListener {

    private static final String TAG = "RinexLogger";
    private static final String FILE_PREFIX = "rinex";
    private static final String ERROR_WRITING_FILE = "Problem writing to file.";
    private static final int MINIMUM_USABLE_FILE_SIZE_BYTES = 1000;

    private final Context mContext;

    private final Object mFileLock = new Object();
    private BufferedWriter mFileWriter;
    private File mFile;

    private boolean firstMes = true;

    private static final int TOW_DECODED_MEASUREMENT_STATE_BIT = 3;
    private static final int C_TO_N0_THRESHOLD_DB_HZ = 18;
    Boolean set_clockbias = false;
    private static double fullBiasNanos = 1.0e-9,BiasNanos = 1.0e-9;

    private HomeFragment.HomeUIFragmentComponent mUiFragmentComponent;


    public synchronized void setUiFragmentComponent(HomeFragment.HomeUIFragmentComponent value) {
        mUiFragmentComponent = value;
    }

    public RinexLogger(Context context) {
        this.mContext = context;
    }

    /** Start a new file logging process. */
    public void startNewLog() {
        synchronized (mFileLock) {
            firstMes=true;
            mUiFragmentComponent.setRinexCounter(0);
            String state = Environment.getExternalStorageState();
            if (Environment.MEDIA_MOUNTED.equals(state)) {
                //baseDirectory = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), FOLDER_PREFIX);
                //baseDirectory.mkdirs();
            } else if (Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
                logError("Cannot write to external storage.");
                return;
            } else {
                logError("Cannot read external storage.");
                return;
            }

            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss");
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

            // To make sure that files do not fill up the external storage:
            // - Remove all empty files
            /**
            FileFilter filter = new RinexLogger.FileToDeleteFilter(mFile);
            for (File existingFile : baseDirectory.listFiles(filter)) {
                existingFile.delete();
            }
            // - Trim the number of files with data
            File[] existingFiles = baseDirectory.listFiles();
            int filesToDeleteCount = existingFiles.length - MAX_FILES_STORED;
            if (filesToDeleteCount > 0) {
                Arrays.sort(existingFiles);
                for (int i = 0; i < filesToDeleteCount; ++i) {
                    existingFiles[i].delete();
                }
            }**/
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


        long gpsTimeNanos = (long) (gnssClock.getTimeNanos() - (gnssClock.getFullBiasNanos() + gnssClock.getBiasNanos()));
        LocalDateTime gpsEpoch = LocalDateTime.of(1980, Month.JANUARY, 6, 0, 0, 0);
        LocalDateTime gpsCalendarTime = gpsEpoch.plusNanos(gpsTimeNanos);

        //OBS
        int numberOfMes = 0;
        StringBuilder obsStringBuilder = new StringBuilder();
        for (GnssMeasurement mes : event.getMeasurements()) {

            if (mes.getConstellationType() != GnssStatus.CONSTELLATION_GPS) {
                continue;
            }

            if (mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                    && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0) {

                /**maintaining constant the 'FullBiasNanos' instead of using the instantaneous value. This avoids the 256 ns
                 jumps each 3 seconds that create a code-phase divergence due to the clock.*/
                if (!set_clockbias) {
                    fullBiasNanos = gnssClock.getFullBiasNanos();
                    BiasNanos = gnssClock.getBiasNanos();
                    set_clockbias = true;
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
        float secondRinex = (float) (gpsCalendarTime.getSecond() + gpsCalendarTime.getNano() / 1e9);
        builder.append(String.format(Locale.US, "> %04d %02d %02d %02d %02d %10.7f %2d %2d%n",
                gpsCalendarTime.getYear(),
                gpsCalendarTime.getMonthValue(),
                gpsCalendarTime.getDayOfMonth(),
                gpsCalendarTime.getHour(),
                gpsCalendarTime.getMinute(),
                secondRinex, 0, numberOfMes));
        // Append OBS
        builder.append(obsStringBuilder);


        if (firstMes) {
            RinexDate startTime = new RinexDate(gpsCalendarTime.getYear(),
                    gpsCalendarTime.getMonthValue(),
                    gpsCalendarTime.getDayOfMonth(),
                    gpsCalendarTime.getHour(),
                    gpsCalendarTime.getMinute(),
                    secondRinex);
            Calendar calendar = Calendar.getInstance();
            String header = rinexHeader(calendar, startTime);
            writeRinexHeader(header);
            System.out.println("RinexLogger|Write Header");
            firstMes = false;
        }
        return builder.toString();
    }

    private String rinexHeader(Calendar calendar, RinexDate startTime) {

        // Date info from the smartphone
        StringBuilder builder = new StringBuilder();

        int year = calendar.get(Calendar.YEAR);
        int month = calendar.get(Calendar.MONTH);
        int day = calendar.get(Calendar.DAY_OF_MONTH);
        int hour = calendar.get(Calendar.HOUR_OF_DAY);
        int minute = calendar.get(Calendar.MINUTE);
        int second = calendar.get(Calendar.SECOND);

        String dateRinex = String.format("%04d%02d%02d", year, month, day);
        String timeRinex = String.format("%02d%02d%02d", hour, minute, second);

        // Header
        String[] header = {"3.03", "", "OBSERVATION DATA", "G: GPS", "RINEX VERSION / TYPE"};
        String[] pgmRunByDate = {"BORLAT", "", dateRinex + " " + timeRinex + " UTC", "PGM / RUN BY / DATE"};
        String[] markerName = {"Unknown", "MARKER NAME"};
        String[] markerNumber = {"Unknown", "MARKER TYPE"};
        String[] observerAgency = {"", "OBSERVER / AGENCY"};
        String[] recTypeVers = {"Unknown", "GnssLogger", "vx.x.x", "REC # / TYPE / VERS"};
        String[] antType = {"", "LEIAS10", "NONE", "ANT # / TYPE"};
        String[] approxPositionAntenna = {"Unknown", "Unknown", "", "", "APPROX POSITION XYZ"};
        String[] antennaDeltas = {"0.0000", "0.0000", "0.0000", "", "ANTENNA: DELTA H/E/N"};
        String[] observationType = {"G", "", "4", " C1C L1C D1C S1C", "SYS / # / OBS TYPES"};

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

        builder.append(String.format(Locale.US,"%6d    %02d    %02d    %02d    %02d%13.7f     %3s         %-20s%n",
                startTime.annee,
                startTime.mois,
                startTime.jour,
                startTime.heure,
                startTime.minute,
                startTime.seconde,
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
                System.out.println("RinexLogger|FileWriter has been closed");
                System.out.println("RinexLogger| Contain :" +mFileWriter);
                System.out.println("RinexLogger|mFile :"+mFile.getPath());
                mFileWriter = null;
            } catch (IOException e) {
                logException("Unable to close all file streams.", e);
            }
        }
    }

    public static class RinexDate {

        private final int annee;
        private final int mois;
        private final int jour;
        private final int heure;
        private final int minute;
        private final float seconde;

        public RinexDate(int annee, int mois, int jour, int heure, int minute, float seconde) {
            this.annee = annee;
            this.mois = mois;
            this.jour = jour;
            this.heure = heure;
            this.minute = minute;
            this.seconde = seconde;
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

    /**
     * Implements a {@link FileFilter} to delete files that are not in the {@link
     * RinexLogger.FileToDeleteFilter#mRetainedFiles}.
     */
    private static class FileToDeleteFilter implements FileFilter {
        private final List<File> mRetainedFiles;

        public FileToDeleteFilter(File... retainedFiles) {
            this.mRetainedFiles = Arrays.asList(retainedFiles);
        }

        /**
         * Returns {@code true} to delete the file, and {@code false} to keep the file.
         *
         * <p>Files are deleted if they are not in the {@link RinexLogger.FileToDeleteFilter#mRetainedFiles} list.
         */
        @Override
        public boolean accept(File pathname) {
            if (pathname == null || !pathname.exists()) {
                return false;
            }
            if (mRetainedFiles.contains(pathname)) {
                return false;
            }
            return pathname.length() < MINIMUM_USABLE_FILE_SIZE_BYTES;
        }
    }
}
