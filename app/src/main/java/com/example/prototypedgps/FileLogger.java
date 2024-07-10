/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.prototypedgps;

import android.content.Context;
import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.location.Location;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.SystemClock;
import android.util.Log;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** A GNSS logger to store information to a file.
 * Source : GNSSLogger*/
public class FileLogger implements MeasurementListener {

    private static final String TAG = "FileLogger";
    private static final String FILE_PREFIX = "raw";
    private static final String ERROR_WRITING_FILE = "Problem writing to file.";
    private static final String COMMENT_START = "# ";
    private static final char RECORD_DELIMITER = ',';
    private static final String VERSION_TAG = "Version: ";

    private final Context mContext;

    private final Object mFileLock = new Object();
    private BufferedWriter mFileWriter;
    private File mFile;

    private HomeFragment.HomeUIFragmentComponent mUiFragmentComponent;


    public synchronized void setUiFragmentComponent(HomeFragment.HomeUIFragmentComponent value) {
        mUiFragmentComponent = value;
    }

    public FileLogger(Context context) {
        this.mContext = context;
    }

    /** Start a new file logging process. */
    public void startNewLog() {
        mUiFragmentComponent.resetRawCounter();
        synchronized (mFileLock) {
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

            // initialize the contents of the file
            try {
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.write("Header Description:");
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.write(VERSION_TAG);
                String manufacturer = Build.MANUFACTURER;
                String model = Build.MODEL;
                String fileVersion =
                        mContext.getString(R.string.app_version)
                                + " Platform: "
                                + Build.VERSION.RELEASE
                                + " "
                                + "Manufacturer: "
                                + manufacturer
                                + " "
                                + "Model: "
                                + model;
                currentFileWriter.write(fileVersion);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.write(
                        "Raw,ElapsedRealtimeMillis,TimeNanos,LeapSecond,TimeUncertaintyNanos,FullBiasNanos,"
                                + "BiasNanos,BiasUncertaintyNanos,DriftNanosPerSecond,DriftUncertaintyNanosPerSecond,"
                                + "HardwareClockDiscontinuityCount,Svid,TimeOffsetNanos,State,ReceivedSvTimeNanos,"
                                + "ReceivedSvTimeUncertaintyNanos,Cn0DbHz,PseudorangeRateMetersPerSecond,"
                                + "PseudorangeRateUncertaintyMetersPerSecond,"
                                + "AccumulatedDeltaRangeState,AccumulatedDeltaRangeMeters,"
                                + "AccumulatedDeltaRangeUncertaintyMeters,CarrierFrequencyHz,CarrierCycles,"
                                + "CarrierPhase,CarrierPhaseUncertainty,MultipathIndicator,SnrInDb,"
                                + "ConstellationType,AgcDb");
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.write(
                        "Fix,Provider,Latitude,Longitude,Altitude,Speed,Accuracy,(UTC)TimeInMs");
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.write("Nav,Svid,Type,Status,MessageId,Sub-messageId,Data(Bytes)");
                currentFileWriter.newLine();
                currentFileWriter.write(COMMENT_START);
                currentFileWriter.newLine();
            } catch (IOException e) {
                logException("Count not initialize file: " + currentFilePath, e);
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

        }
    }

    /**
     * Send the current log via email or other options selected from a pop menu shown to the user. A
     * new log is started when calling this function.
     */
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

    @Override
    public void onProviderEnabled(String provider) {}

    @Override
    public void onProviderDisabled(String provider) {}

    @Override
    public void onLocationChanged(Location location) {
        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            String locationStream =
                    String.format(
                            Locale.US,
                            "Fix,%s,%f,%f,%f,%f,%f,%d",
                            location.getProvider(),
                            location.getLatitude(),
                            location.getLongitude(),
                            location.getAltitude(),
                            location.getSpeed(),
                            location.getAccuracy(),
                            location.getTime());
            try {
                mFileWriter.write(locationStream);
                mFileWriter.newLine();
                if (mUiFragmentComponent != null) {
                    mUiFragmentComponent.incrementRawCounter();
                }
            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }
    }

    @Override
    public void onLocationStatusChanged(String provider, int status, Bundle extras) {}

    @Override
    public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {
        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            GnssClock gnssClock = event.getClock();
            for (GnssMeasurement measurement : event.getMeasurements()) {
                try {
                    writeGnssMeasurementToFile(gnssClock, measurement);
                    if (mUiFragmentComponent != null) {
                        mUiFragmentComponent.incrementRawCounter();
                    }
                } catch (IOException e) {
                    logException(ERROR_WRITING_FILE, e);
                }
            }
        }
    }

    @Override
    public void onGnssMeasurementsStatusChanged(int status) {}

    @Override
    public void onGnssNavigationMessageReceived(GnssNavigationMessage navigationMessage) {
        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            StringBuilder builder = new StringBuilder("Nav");
            builder.append(RECORD_DELIMITER);
            builder.append(navigationMessage.getSvid());
            builder.append(RECORD_DELIMITER);
            builder.append(navigationMessage.getType());
            builder.append(RECORD_DELIMITER);

            int status = navigationMessage.getStatus();
            builder.append(status);
            builder.append(RECORD_DELIMITER);
            builder.append(navigationMessage.getMessageId());
            builder.append(RECORD_DELIMITER);
            builder.append(navigationMessage.getSubmessageId());
            byte[] data = navigationMessage.getData();
            for (byte word : data) {
                builder.append(RECORD_DELIMITER);
                builder.append(word);
            }
            try {
                mFileWriter.write(builder.toString());
                mFileWriter.newLine();
                if (mUiFragmentComponent != null) {
                    mUiFragmentComponent.incrementRawCounter();
                }

            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }
    }

    @Override
    public void onGnssNavigationMessageStatusChanged(int status) {}

    @Override
    public void onGnssStatusChanged(GnssStatus gnssStatus) {}

    @Override
    public void onNmeaReceived(long timestamp, String s) {
        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            String nmeaStream = String.format(Locale.US, "NMEA,%s,%d", s.trim(), timestamp);
            try {
                mFileWriter.write(nmeaStream);
                mFileWriter.newLine();
                if (mUiFragmentComponent != null) {
                    mUiFragmentComponent.incrementRawCounter();
                }

            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }
    }

    @Override
    public void onListenerRegistration(String listener, boolean result) {}

    @Override
    public void onTTFFReceived(long l) {}

    private void writeGnssMeasurementToFile(GnssClock clock, GnssMeasurement measurement)
            throws IOException {
        String clockStream =
                String.format(
                        "Raw,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                        SystemClock.elapsedRealtime(),
                        clock.getTimeNanos(),
                        clock.hasLeapSecond() ? clock.getLeapSecond() : "",
                        clock.hasTimeUncertaintyNanos() ? clock.getTimeUncertaintyNanos() : "",
                        clock.getFullBiasNanos(),
                        clock.hasBiasNanos() ? clock.getBiasNanos() : "",
                        clock.hasBiasUncertaintyNanos() ? clock.getBiasUncertaintyNanos() : "",
                        clock.hasDriftNanosPerSecond() ? clock.getDriftNanosPerSecond() : "",
                        clock.hasDriftUncertaintyNanosPerSecond()
                                ? clock.getDriftUncertaintyNanosPerSecond()
                                : "",
                        clock.getHardwareClockDiscontinuityCount() + ",");
        mFileWriter.write(clockStream);

        String measurementStream =
                String.format(
                        "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                        measurement.getSvid(),
                        measurement.getTimeOffsetNanos(),
                        measurement.getState(),
                        measurement.getReceivedSvTimeNanos(),
                        measurement.getReceivedSvTimeUncertaintyNanos(),
                        measurement.getCn0DbHz(),
                        measurement.getPseudorangeRateMetersPerSecond(),
                        measurement.getPseudorangeRateUncertaintyMetersPerSecond(),
                        measurement.getAccumulatedDeltaRangeState(),
                        measurement.getAccumulatedDeltaRangeMeters(),
                        measurement.getAccumulatedDeltaRangeUncertaintyMeters(),
                        measurement.hasCarrierFrequencyHz() ? measurement.getCarrierFrequencyHz() : "",
                        measurement.hasCarrierCycles() ? measurement.getCarrierCycles() : "",
                        measurement.hasCarrierPhase() ? measurement.getCarrierPhase() : "",
                        measurement.hasCarrierPhaseUncertainty()
                                ? measurement.getCarrierPhaseUncertainty()
                                : "",
                        measurement.getMultipathIndicator(),
                        measurement.hasSnrInDb() ? measurement.getSnrInDb() : "",
                        measurement.getConstellationType(),
                        measurement.hasAutomaticGainControlLevelDb()
                                ? measurement.getAutomaticGainControlLevelDb()
                                : "");
        mFileWriter.write(measurementStream);
        mFileWriter.newLine();

    }

    private void logException(String errorMessage, Exception e) {
        Log.e(MeasurementProvider.TAG + TAG, errorMessage, e);
        Toast.makeText(mContext, errorMessage, Toast.LENGTH_LONG).show();
    }

    private void logError(String errorMessage) {
        Log.e(MeasurementProvider.TAG + TAG, errorMessage);
        Toast.makeText(mContext, errorMessage, Toast.LENGTH_LONG).show();
    }


}
