package com.example.prototypedgps;

import android.content.Context;
import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.location.Location;
import android.location.cts.nano.Ephemeris;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.util.Pair;
import android.widget.Toast;

import com.example.pseudorange.EphemerisManager;
import com.example.pseudorange.GpsTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

public class RealTimePositionCalculator implements MeasurementListener {

    private static final int TOW_DECODED_MEASUREMENT_STATE_BIT = 3;
    private static final int C_TO_N0_THRESHOLD_DB_HZ = 18;

    private Boolean setClockBias = false;

    private static double fullBiasNanos = 1.0e-9, BiasNanos = 1.0e-9;

    private HomeFragment.HomeUIFragmentComponent mUiFragmentComponent;

    private StatusFragment.StatusUIFragmentComponent mStatusUIFragmentComponent;

    private final BaseStation mBaseStation;

    private final EphemerisManager mEphemerisManager;

    private GnssStatus mGnssStatus;

    private final Object mFileLock = new Object();

    private final Object mFileLockConstraint = new Object();

    private static final String FILE_PREFIX = "results";

    private static final String FILE_PREFIX_CONSTRAINT = "results_constraint";

    private static final String TAG = "ResultsLogger";

    private static final String ERROR_WRITING_FILE = "Problem writing to file.";

    private BufferedWriter mFileWriter;

    private BufferedWriter mFileWriterConstraint;


    private File mFile;

    private File mFileConstraint;

    private final Context mContext;

    public RealTimePositionCalculator(BaseStation baseStation, EphemerisManager ephemerisManager, Context context) {

        this.mBaseStation = baseStation;
        this.mEphemerisManager = ephemerisManager;
        this.mContext = context;
    }

    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {

    }

    @Override
    public void onLocationChanged(Location location) {

        JSONObject json = new JSONObject();
        try {
            json.put("type","Location");
            json.put("latitude",location.getLatitude());
            json.put("longitude",location.getLongitude());
            json.put("height",location.getAltitude());
            json.put("time",location.getTime());
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        if(mUiFragmentComponent.isSendUDPChecked()){
        sendResultsUDP(json);
        }
    }

    @Override
    public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {

        // Wait 1 half second for the last RTCM messages to be decoded
        try {
            Thread.sleep(500); // 1000 millisecond = 1 second
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        Receiver mBaseStationReceiver = mBaseStation.getmBaseStationReceiver();
        computeDGPSSingleEpoch(event, mBaseStationReceiver);

    }

    @Override
    public void onGnssMeasurementsStatusChanged(int status) {


    }

    @Override
    public void onLocationStatusChanged(String provider, int status, Bundle extras) {

    }


    @Override
    public void onGnssNavigationMessageReceived(GnssNavigationMessage navigationMessage) {
        if (navigationMessage.getType() == GnssNavigationMessage.TYPE_GPS_L1CA) {
            System.out.println("MeasurementProvider| GnssNavigationMessage received");
            mEphemerisManager.parseHwNavigationMessageUpdates(navigationMessage);
        }
    }

    @Override
    public void onGnssNavigationMessageStatusChanged(int status) {

    }

    @Override
    public void onGnssStatusChanged(GnssStatus gnssStatus) {
        mGnssStatus = gnssStatus;
        if (mStatusUIFragmentComponent != null) {
            ArrayList<Map<String,String>> allGpsSat = new ArrayList<>();
            for (int i = 0; i < gnssStatus.getSatelliteCount(); i++) {
                // Get only GPS sat
                if (gnssStatus.getConstellationType(i) == GnssStatus.CONSTELLATION_GPS) {
                    Map<String, String> gpsSat = new HashMap<>();
                    int svId = gnssStatus.getSvid(i);
                    float cn0DbHz = gnssStatus.getCn0DbHz(i);
                    float elevation = gnssStatus.getElevationDegrees(i);
                    float azimuth = gnssStatus.getAzimuthDegrees(i);
                    gpsSat.put("svId", String.valueOf(svId));
                    gpsSat.put("cn0DbHz", String.valueOf(cn0DbHz));
                    gpsSat.put("elevation", String.valueOf(elevation));
                    gpsSat.put("azimuth", String.valueOf(azimuth));
                    allGpsSat.add(gpsSat);
                }

            }
            mStatusUIFragmentComponent.updateStatusTable(allGpsSat);

        }
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


    private void computeDGPSSingleEpoch(GnssMeasurementsEvent event, Receiver mBaseStationReceiver) {

        // Position of base station
        if (mBaseStation.isStationaryAntennaDecoded()) {
            mUiFragmentComponent.stationaryAntennaDecoded();
        } else {
            System.out.println("computeDGPSSingleEpoch| Position of base station not decoded");
            return;
        }

        // Epoch of position computation
        GnssClock gnssClock = event.getClock();

        // Check that the receiver have estimate GPS time
        if (!gnssClock.hasFullBiasNanos()) {
            System.out.println("computeDGPSSingleEpoch| FullBiasNanos is not decoded, exit positioning calculation");
            return;
        }

        double gpsTimeNanos = gnssClock.getTimeNanos() - (gnssClock.getFullBiasNanos() + gnssClock.getBiasNanos());
        TimeE timeOfRover = new TimeE(gpsTimeNanos / 1e6);

        if(mUiFragmentComponent.isSendUDPChecked()){
            JSONObject json = new JSONObject();
            try {
                json.put("type","Mes");
                json.put("epoch",gpsTimeNanos);
                JSONArray jsonArraySatId = new JSONArray();
                JSONArray jsonArrayNoise = new JSONArray();
                for (GnssMeasurement mes : event.getMeasurements()) {

                    if (mes.getConstellationType() == GnssStatus.CONSTELLATION_GPS) {
                        jsonArraySatId.put(mes.getSvid());
                        jsonArrayNoise.put(String.format(Locale.US, "%.3f", mes.getCn0DbHz()));
                    }
                }
                json.put("satId",jsonArraySatId);
                json.put("noise",jsonArrayNoise);

            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            sendResultsUDP(json);
        }

        // Base station observations
        ArrayList<Observations> observationsArrayList = mBaseStationReceiver.getEpochsArrayList();

        // Exit if not enough GNSS measurements or base station observations
        if (event.getMeasurements().size() < 4) {
            System.out.println("computeDGPSSingleEpoch| Number of gnssMeasurements observations < 4 , exit positioning calculation");
            return;
        }
        if (observationsArrayList.size() < 4) {
            System.out.println("computeDGPSSingleEpoch| Number of decoded base station observations < 4 , exit positioning calculation");
            return;
        }

        // Search base station closest observation
        double deltaMin = 100000;
        double timeNear = 0;
        double timeBefor = 0;

        Observations baseStationBeforNearObservations = null;
        Observations baseStationNearObservations = null;

        for (int i = observationsArrayList.size() - 1; i >= 0; i--) {

            Observations iObservation = observationsArrayList.get(i);
            Time iTime = iObservation.getRefTime();
            double iGpsTime = iTime.getMsecGpsTime();
            System.out.println("computeDGPSSingleEpoch| currentGpsTime: " + iTime.getMsecGpsTime());
            double iDelta = Math.abs(gpsTimeNanos / 1e6 - iGpsTime);

            if (iDelta > deltaMin) {

                baseStationNearObservations = observationsArrayList.get(i + 1);
                timeNear = baseStationNearObservations.getRefTime().getMsecGpsTime() / 1e3;
                baseStationBeforNearObservations = observationsArrayList.get(i);
                timeBefor = baseStationBeforNearObservations.getRefTime().getMsecGpsTime() / 1e3;
                System.out.printf("computeDGPSSingleEpoch| closest: %.1f\n", timeNear);  // Print with 6 decimal places
                System.out.printf("computeDGPSSingleEpoch| before closest: %.1f\n", timeBefor);
                System.out.printf("computeDGPSSingleEpoch| timeRover (closest): %.1f\n", iGpsTime);

                break; // Get out of the loop when iDelta > deltaMin
            } else {
                deltaMin = iDelta;
            }
        }

        if (deltaMin > 1000.0) {
            System.out.println("computeDGPSSingleEpoch| deltaMin>1s, exit positioning calculation");
            return;
        }

        System.out.printf("computeDGPSSingleEpoch| delta (msec):  %.0f\n", deltaMin);

        // Initialization lists of available svId
        ArrayList<Integer> svIdRover = new ArrayList<>();
        ArrayList<Integer> svIdBaseStation = new ArrayList<>();
        ArrayList<Integer> commonSvId = new ArrayList<>();

        int nbrSatObserved = event.getMeasurements().size();
        int nbrSatObservedGps = 0;

        // Available svId rover
        for (GnssMeasurement mes : event.getMeasurements()) {

            if (mes.getConstellationType() == GnssStatus.CONSTELLATION_GPS) {
                nbrSatObservedGps += 1;
                if (mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                        && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0
                        && getElevationFromGpsSvId(mes.getSvid()) >= 15.0) {
                    svIdRover.add(mes.getSvid());
                }
            }
        }

        // Available svId base station
        for (ObservationSet obs : baseStationNearObservations.getObsSet()) {
            svIdBaseStation.add(obs.getSatID());
        }

        // Common available svId + check if ephemeris is available
        for (Integer value : svIdRover) {
            if (svIdBaseStation.contains(value)
                    && hasEphemeris(value)) {
                commonSvId.add(value);
            }
        }

        if (commonSvId.size() < 4) {
            System.out.println("computeDGPSSingleEpoch| Number of common available sat < 4 , exit positioning calculation");
            return;
        }
        System.out.println("computeDGPSSingleEpoch| common svId : " + commonSvId);

        ArrayList<DoubleDiff> doubleDiffArrayList = new ArrayList<>();

        // Search for the most zenith satellite
        int refSv = 0;
        float elevationRefSv = 0;
        for (Integer sv : commonSvId) {

            for (int i = 0; i < mGnssStatus.getSatelliteCount(); i++) {
                if (mGnssStatus.getConstellationType(i) == GnssStatus.CONSTELLATION_GPS
                        && mGnssStatus.getSvid(i) == sv
                        && mGnssStatus.getElevationDegrees(i) > elevationRefSv
                ) {
                    refSv = mGnssStatus.getSvid(i);
                    elevationRefSv = mGnssStatus.getElevationDegrees(i);
                }
            }

        }

        ObservationSet observationSetRefSv = null;
        ObservationSet observationSetRefSvBefor = null;

        int idObs = 0;
        for (Integer sv : commonSvId) {

            double pseudorangeSvBaseStation = 0;
            double pseudorangeSvBaseStationBefor = 0;
            double pseudorangeSvRover = 0;
            double pseudorangeSvRefRover = 0;

            // Compute rover pseudorange
            for (GnssMeasurement mes : event.getMeasurements()) {
                if (mes.getConstellationType() == GnssStatus.CONSTELLATION_GPS
                        && hasEphemeris(mes.getSvid())
                        && mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                        && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0) {
                    if (mes.getSvid() == sv) {
                        pseudorangeSvRover = computePseudorange(mes, gnssClock);

                    }
                    if (mes.getSvid() == refSv) {
                        pseudorangeSvRefRover = computePseudorange(mes, gnssClock);
                    }
                }

            }

            // Retrieve base station pseudorange
            for (ObservationSet obs : baseStationNearObservations.getObsSet()) {
                if (obs.getSatID() == sv) {
                    pseudorangeSvBaseStation = obs.getPseudorange(0);
                }
                if (obs.getSatID() == refSv) {
                    observationSetRefSv = obs;
                }
            }

            for (ObservationSet obs : baseStationBeforNearObservations.getObsSet()) {
                if (obs.getSatID() == sv) {
                    pseudorangeSvBaseStationBefor = obs.getPseudorange(0);
                }
                if (obs.getSatID() == refSv) {
                    observationSetRefSvBefor = obs;
                }
            }

            if (refSv != sv) {

                System.out.println("computeDGPSSingleEpoch| form double diff : (" + refSv + "," + sv + ")");

                // Sv
                // Differential and asynchronous GNSS processing
                double doppler = (pseudorangeSvBaseStation - pseudorangeSvBaseStationBefor) / (timeNear - timeBefor);
                double pseudorangeSvBaseStationSync = pseudorangeSvBaseStation + doppler * deltaMin / 1000 * Constants.SPEED_OF_LIGHT / Constants.FL1;
                System.out.printf("computeDGPSSingleEpoch| pseudorange sat %s [Base(Sync), Rover]: [%f,%f]\n", sv, pseudorangeSvBaseStationSync, pseudorangeSvRover);
                double simpleDiffSv = pseudorangeSvBaseStationSync - pseudorangeSvRover;

                // SvRef
                double dopplerRef = (observationSetRefSv.getPseudorange(0) - observationSetRefSvBefor.getPseudorange(0)) / (timeNear - timeBefor);
                System.out.println("computeDGPSSingleEpoch| timeNear-timeBefor : " + (timeNear - timeBefor));
                double pseudorangeRefSvBaseStationSyncro = observationSetRefSv.getPseudorange(0) + dopplerRef * deltaMin / 1000 * Constants.SPEED_OF_LIGHT / Constants.FL1;
                double simpleDiffRef = pseudorangeRefSvBaseStationSyncro - pseudorangeSvRefRover;
                System.out.printf("computeDGPSSingleEpoch| pseudorange sat ref %s [Base, Rover]: [%f,%f]\n", refSv, pseudorangeRefSvBaseStationSyncro, pseudorangeSvRefRover);

                double doubleDiff = simpleDiffRef - simpleDiffSv;
                System.out.println("computeDGPSSingleEpoch| double difference : " + doubleDiff);

                DoubleDiff doubleDiffObs = new DoubleDiff(refSv, sv);
                doubleDiffObs.zeroDiffBase.put(0, pseudorangeRefSvBaseStationSyncro);
                doubleDiffObs.zeroDiffBase.put(1, pseudorangeSvBaseStationSync);
                doubleDiffObs.zeroDiffRover.put(0, pseudorangeSvRefRover);
                doubleDiffObs.zeroDiffRover.put(1, pseudorangeSvRover);
                doubleDiffObs.doubleDiff = doubleDiff;
                doubleDiffObs.elevationPRN1 = getElevationFromGpsSvId(doubleDiffObs.PRN1);
                doubleDiffObs.elevationPRN2 = getElevationFromGpsSvId(doubleDiffObs.PRN2);

                idObs += 1;
                doubleDiffObs.iObs = idObs;
                doubleDiffArrayList.add(doubleDiffObs);
            }
        }

        if (doubleDiffArrayList.size() < 4) {
            return;
        }

        if(mUiFragmentComponent.isComputeConstraint()){
            RealMatrix constraintPoint = mUiFragmentComponent.getCoordinateConstrainedPoint();
            computePosConstraint(doubleDiffArrayList, timeOfRover, nbrSatObserved, nbrSatObservedGps, constraintPoint);
        }
        computePosBiber(doubleDiffArrayList, timeOfRover, nbrSatObserved, nbrSatObservedGps);
        computePos(doubleDiffArrayList, timeOfRover, nbrSatObserved, nbrSatObservedGps);

    }

    private float getElevationFromGpsSvId(int svId) {

        for (int i = 0; i < mGnssStatus.getSatelliteCount(); i++) {
            if (mGnssStatus.getConstellationType(i) == GnssStatus.CONSTELLATION_GPS
                    && mGnssStatus.getSvid(i) == svId
            ) {
                return mGnssStatus.getElevationDegrees(i);
            }
        }
        return 0.0F;
    }

    private void computePosBiber(ArrayList<DoubleDiff> doubleDiffArrayList, TimeE timeOfRover, int nbrSatObserved, int nbrObservationGps) {

        // Class to storage the results
        Results res = new Results();

        // Parameters
        // A priori error on observation [m]
        double observationStandardDeviation = 0.3;
        // Biber constant to construct matrix of weight W
        double constantMEstimator = 2.5;

        // Get position of base station
        RealMatrix X_Base = mBaseStation.getStationaryAntenna();
        // Get current Ephemeris
        Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

        // Number of observation and unknown
        int nbrObservations = doubleDiffArrayList.size();
        int nbrUnknowns = 3;

        // Set approximate values for unknown parameters as coordinates of base station
        RealMatrix X_Rover = X_Base;
        RealMatrix X_Rover_previous = X_Base;

        // Initialization of the matrix
        RealMatrix l = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix f0 = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix A = new BlockRealMatrix(nbrObservations, nbrUnknowns);

        RealMatrix dl = null;
        RealMatrix dx = null;
        RealMatrix v = null;
        RealMatrix Qll = null;
        RealMatrix P = null;
        RealMatrix Qxx = null;

        // Stochastic model
        double sigma0 = 1.0;
        RealMatrix Kll = new BlockRealMatrix(nbrObservations, nbrObservations);

        // Weight matrix M-estimator, initialize as identity matrix for iteration 0
        RealMatrix W = MatrixUtils.createRealIdentityMatrix(nbrObservations);

        // Compute iteration M-estimator (Biber)
        boolean contMEstimator = true;
        int nbr_iteration_M_estimator = 0;

        while (contMEstimator) {

            // Compute iteration least squares estimation
            boolean cont = true;
            int nbr_iteration = 0;

            while (cont) {

                // Fill the matrix by going through each observation
                int idObs = 0;
                for (DoubleDiff doubleDiff : doubleDiffArrayList) {

                    double doubleDiffVal = doubleDiff.getDoubleDiff();

                    // Fill matrix l
                    l.setEntry(idObs, 0, doubleDiffVal);

                    // Fill matrix A by numerical derivation of the observation equation
                    double f = computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);

                    // Component x
                    RealMatrix X_Rover_dx = X_Rover.copy();
                    X_Rover_dx.addToEntry(0, 0, 1.0);
                    double f_dx = computeObsEquation(X_Rover_dx, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 0, f_dx - f);

                    // Component y
                    RealMatrix X_Rover_dy = X_Rover.copy();
                    X_Rover_dy.addToEntry(1, 0, 1.0);
                    double f_dy = computeObsEquation(X_Rover_dy, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 1, f_dy - f);

                    // Component z
                    RealMatrix X_Rover_dz = X_Rover.copy();
                    X_Rover_dz.addToEntry(2, 0, 1.0);
                    double f_dz = computeObsEquation( X_Rover_dz, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 2, f_dz - f);

                    // Fill f0
                    f0.setEntry(idObs, 0, computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base));

                    // Fill Kll
                    Kll.setEntry(idObs,idObs,observationStandardDeviation);

                    idObs += 1;
                }

                Qll = Kll.scalarMultiply(1.0 / (sigma0 * sigma0));
                P = new SingularValueDecomposition(Qll).getSolver().getInverse();
                dl = l.subtract(f0);

                dx = new SingularValueDecomposition(A.transpose().multiply(W).multiply(P).multiply(A)).getSolver().getInverse().multiply(A.transpose()).multiply(W).multiply(P).multiply(dl);

                // Update unknown approximate vector
                X_Rover = X_Rover.add(dx);

                // Interruption criteria
                double maxNorm = 0.0;
                for (int i = 0; i < dx.getRowDimension(); i++) {
                    double absValue = Math.abs(dx.getEntry(i, 0));
                    if (absValue > maxNorm) {
                        maxNorm = absValue;
                    }
                }

                if (maxNorm <= 1e-3) {
                    cont = false;
                }
                if (nbr_iteration >= 20){
                    return;
                }

                // Update iteration for convergence of dx
                nbr_iteration++;

            }

            // Update iteration for convergence of M-estimator
            nbr_iteration_M_estimator++;

            // Compute residuals v
            v = A.multiply(dx).subtract(dl);
            RealMatrix N = A.transpose().multiply(P).multiply(A);
            // Cofactor matrix of estimated parameters
            Qxx = new SingularValueDecomposition(N).getSolver().getInverse();
            RealMatrix Qvv = Qll.subtract(A.multiply(Qxx).multiply(A.transpose()));

            // Compute standardized residuals w
            RealMatrix w = new BlockRealMatrix(nbrObservations, 1);
            for (int i = 0; i < nbrObservations; i++) {
                w.setEntry(i, 0, v.getEntry(i,0)/(sigma0 * Math.sqrt(Qvv.getEntry(i, i))));
            }

            // Fill weight matrix W
            for (int j = 0; j < nbrObservations; j++) {
                if (Math.abs(w.getEntry(j, 0)) < constantMEstimator) {
                    W.setEntry(j, j, 1.0);
                } else {
                    W.setEntry(j, j, constantMEstimator / Math.abs(w.getEntry(j, 0)));
                }
            }

            // Difference between current and previous parameter vector
            RealMatrix dX_Rover = X_Rover.subtract(X_Rover_previous);

            // Interruption criteria
            double maxNorm = 0.0;
            for (int i = 0; i < dX_Rover.getRowDimension(); i++) {
                double absValue = Math.abs(dX_Rover.getEntry(i, 0));
                if (absValue > maxNorm) {
                    maxNorm = absValue;
                }
            }

            if (nbr_iteration_M_estimator >= 20) {
                return;
            }
            if (maxNorm <= 1e-3){
                contMEstimator = false;
            }

            // Storage of current parameter vector
            X_Rover_previous = X_Rover;

        }

        // Statistical indicators

        // Empirical standard deviation of unit weight
        double s0 = Math.sqrt(v.transpose().multiply(P).multiply(v).getEntry(0, 0) / (nbrObservations - nbrUnknowns));
        // Variance-covariance matrix of estimated parameters
        RealMatrix Kxx = Qxx.scalarMultiply(s0);

        //Indicators dilution of precision (DOPs)
        double GDOP = Math.sqrt(Qxx.getTrace());
        double PDOP = Math.sqrt(Qxx.getTrace());

        // Param ellipsoid Bessel
        double[] ellParam = ellBesselParam();
        double[] ellCoordinate = cart2ell(Constants.ELL_A_GRS80, ellParam[0], X_Rover);

        double lat0 = Math.toRadians(ellCoordinate[1]);
        double lon0 = Math.toRadians(ellCoordinate[0]);

        double sinLat0 = Math.sin(lat0);
        double cosLat0 = Math.cos(lat0);
        double sinLon0 = Math.sin(lon0);
        double cosLon0 = Math.cos(lon0);

        double[][] rotation = new double[][]{
                {-sinLat0 * cosLon0, -sinLat0 * sinLon0, cosLat0},
                {-sinLon0, cosLon0, 0},
                {cosLat0 * cosLon0, cosLat0 * sinLon0, sinLat0}
        };

        RealMatrix matrixR = new Array2DRowRealMatrix(rotation);

        RealMatrix matrixTopo = matrixR.multiply(Qxx.multiply(matrixR.transpose()));

        // HDOP and VDOP
        double HDOP = Math.sqrt(matrixTopo.getEntry(0, 0) + matrixTopo.getEntry(1, 1));
        double VDOP = Math.sqrt(matrixTopo.getEntry(2, 2));

        // Variance-covariance matrix of estimated parameters in topocentric system
        RealMatrix KxxTopo = matrixR.multiply(Kxx.multiply(matrixR.transpose()));
        double sigmaEast = Math.sqrt(KxxTopo.getEntry(1,1));
        double sigmaNorth = Math.sqrt(KxxTopo.getEntry(0,0));
        double sigmaHeight = Math.sqrt(KxxTopo.getEntry(2,2));

        // Update res in UI
        double[] MN95 = CHTRS95toMN95(X_Rover);
        mUiFragmentComponent.updateEastNorthHBessel(MN95[0], MN95[1], MN95[2]);
        mUiFragmentComponent.updateRes(nbrSatObserved, nbrObservationGps, nbrObservations, PDOP, VDOP, HDOP, (s0/sigma0), sigmaEast, sigmaNorth, sigmaHeight);
        mUiFragmentComponent.updateDeltaGroundTrue(X_Rover);

        // Save data in Result class
        res.s0 = s0;
        res.sigma0 = sigma0;
        res.X_Rover = X_Rover;
        res.Kxx = Kxx;
        res.GDOP = GDOP;
        res.PDOP = PDOP;
        res.VDOP = VDOP;
        res.HDOP = HDOP;
        res.Kll = Kll;
        res.doubleDiffArrayList = doubleDiffArrayList;
        res.v = v;
        res.nbrSatObserved = nbrSatObserved;
        res.nbrObservationsGps = nbrObservationGps;
        res.sigmaEast = sigmaEast;
        res.sigmaNorth = sigmaNorth;
        res.sigmaHeight = sigmaHeight;
        res.nbrObs = nbrObservations;
        res.nbrIteration = nbr_iteration_M_estimator;
        res.nbrUnknowns = nbrUnknowns;
        res.epoch = timeOfRover;

        if(mUiFragmentComponent.isSendUDPChecked()){
        JSONObject json;
        try {
            json = res.toJSON("ResBiber");
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
        sendResultsUDP(json);
        }

        synchronized (mFileLock) {
            if (mFileWriter == null) {
                return;
            }
            try {
                String result = res.resultToString(false, mBaseStation);
                mFileWriter.write(result);

                if (mUiFragmentComponent != null) {
                    mUiFragmentComponent.incrementDetailedResultsCounter();
                }
            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }
    }

    private void computePos(ArrayList<DoubleDiff> doubleDiffArrayList, TimeE timeOfRover, int nbrSatObserved, int nbrObservationGps) {

        // Class to storage the results
        Results res = new Results();

        // Parameters
        // A priori error on observation [m]
        double observationStandardDeviation = 0.3;

        // Get position of base station
        RealMatrix X_Base = mBaseStation.getStationaryAntenna();
        // Get current Ephemeris
        Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

        // Number of observation and unknowns
        int nbrObservations = doubleDiffArrayList.size() + 3;
        int nbrUnknowns = 3;

        // Set approximate values for unknown parameters as coordinates of base station
        RealMatrix X_Rover = X_Base;

        // Initialization of the matrix
        RealMatrix l = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix f0 = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix A = new BlockRealMatrix(nbrObservations, nbrUnknowns);

        RealMatrix dx = null;
        RealMatrix dl = null;
        RealMatrix P = null;

        // Stochastic model
        double sigma0 = 1.0;
        RealMatrix Kll = new BlockRealMatrix(nbrObservations, nbrObservations);

        // Compute iteration
        boolean cont = true;
        int nbr_iteration = 0;

        while (cont) {

            // Fill the matrix by going through each observation
            int idObs = 0;
            for (DoubleDiff doubleDiff : doubleDiffArrayList) {

                double doubleDiffVal = doubleDiff.getDoubleDiff();

                // Fill matrix l
                l.setEntry(idObs, 0, doubleDiffVal);

                // Fill matrix A by numerical derivation of the observation equation
                double f = computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);

                // Component x
                RealMatrix X_Rover_dx = X_Rover.copy();
                X_Rover_dx.addToEntry(0, 0, 1.0);
                double f_dx = computeObsEquation(X_Rover_dx, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 0, f_dx - f);

                // Component y
                RealMatrix X_Rover_dy = X_Rover.copy();
                X_Rover_dy.addToEntry(1, 0, 1.0);
                double f_dy = computeObsEquation(X_Rover_dy, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 1, f_dy - f);

                // Component z
                RealMatrix X_Rover_dz = X_Rover.copy();
                X_Rover_dz.addToEntry(2, 0, 1.0);
                double f_dz = computeObsEquation(X_Rover_dz, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 2, f_dz - f);

                // Fill f0
                f0.setEntry(idObs, 0, computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base));

                // Fill Kll
                Kll.setEntry(idObs,idObs,observationStandardDeviation);

                idObs += 1;
            }

            RealMatrix Qll = Kll.scalarMultiply(1 / (sigma0 * sigma0));
            P = new SingularValueDecomposition(Qll).getSolver().getInverse();
            dl = l.subtract(f0);

            dx = new SingularValueDecomposition(A.transpose().multiply(P).multiply(A)).getSolver().getInverse().multiply(A.transpose()).multiply(P).multiply(dl);

            // Update unknown approximate vector
            X_Rover = X_Rover.add(dx);

            // Interruption criteria
            double maxNorm = 0.0;
            for (int i = 0; i < dx.getRowDimension(); i++) {
                double absValue = Math.abs(dx.getEntry(i, 0));
                if (absValue > maxNorm) {
                    maxNorm = absValue;
                }
            }

            if (maxNorm <= 1e-3) {
                cont = false;
            }
            if (nbr_iteration > 10){
                res.status = "no convergence, number of iteration > 10";
                cont = false;
            }

            // Update iteration for convergence of dx
            nbr_iteration++;
        }

        // Compute residuals v
        RealMatrix v = A.multiply(dx).subtract(dl);

        // Cofactor matrix of estimated parameters
        RealMatrix Qxx = new SingularValueDecomposition(A.transpose().multiply(P).multiply(A)).getSolver().getInverse();

        // Empirical standard deviation of unit weight
        double s0 = Math.sqrt(v.transpose().multiply(P).multiply(v).getEntry(0, 0) / (nbrObservations - nbrUnknowns));
        // Variance-covariance matrix of estimated parameters
        RealMatrix Kxx = Qxx.scalarMultiply(s0);

        //Indicators dilution of precision (DOPs)
        double GDOP = Math.sqrt(Qxx.getTrace());
        double PDOP = Math.sqrt(Qxx.getTrace());

        // Param ellipsoid Bessel
        double[] ellParam = ellBesselParam();
        double[] ellCoordinate = cart2ell(Constants.ELL_A_GRS80, ellParam[0], X_Rover);

        double lat0 = Math.toRadians(ellCoordinate[1]);
        double lon0 = Math.toRadians(ellCoordinate[0]);

        double sinLat0 = Math.sin(lat0);
        double cosLat0 = Math.cos(lat0);
        double sinLon0 = Math.sin(lon0);
        double cosLon0 = Math.cos(lon0);

        double[][] rotation = new double[][]{
                {-sinLat0 * cosLon0, -sinLat0 * sinLon0, cosLat0},
                {-sinLon0, cosLon0, 0},
                {cosLat0 * cosLon0, cosLat0 * sinLon0, sinLat0}
        };

        RealMatrix matrixR = new Array2DRowRealMatrix(rotation);

        RealMatrix matrixTopo = matrixR.multiply(Qxx.multiply(matrixR.transpose()));

        // HDOP and VDOP
        double HDOP = Math.sqrt(matrixTopo.getEntry(0, 0) + matrixTopo.getEntry(1, 1));
        double VDOP = Math.sqrt(matrixTopo.getEntry(2, 2));

        // Variance-covariance matrix of estimated parameters in topocentric system
        RealMatrix KxxTopo = matrixR.multiply(Kxx.multiply(matrixR.transpose()));
        double sigmaEast = Math.sqrt(KxxTopo.getEntry(1,1));
        double sigmaNorth = Math.sqrt(KxxTopo.getEntry(0,0));
        double sigmaHeight = Math.sqrt(KxxTopo.getEntry(2,2));

        // Save data in Result class
        res.s0 = s0;
        res.sigma0 = sigma0;
        res.X_Rover = X_Rover;
        res.Kxx = Kxx;
        res.GDOP = GDOP;
        res.PDOP = PDOP;
        res.VDOP = VDOP;
        res.HDOP = HDOP;
        res.Kll = Kll;
        res.doubleDiffArrayList = doubleDiffArrayList;
        res.v = v;
        res.nbrSatObserved = nbrSatObserved;
        res.nbrObservationsGps = nbrObservationGps;
        res.sigmaEast = sigmaEast;
        res.sigmaNorth = sigmaNorth;
        res.sigmaHeight = sigmaHeight;

        if(mUiFragmentComponent.isSendUDPChecked()){
            JSONObject json;
            try {
                json = res.toJSON("Res");
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            sendResultsUDP(json);
        }
    }

    private void computePosConstraint(ArrayList<DoubleDiff> doubleDiffArrayList, TimeE timeOfRover, int nbrSatObserved, int nbrObservationGps, RealMatrix constraintPoint) {

        // Class to storage the results
        Results res = new Results();

        // Parameters
        double observationStandardDeviation = 0.3;

        // Get position of base station
        RealMatrix X_Base = mBaseStation.getStationaryAntenna();
        // Get current Ephemeris
        Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

        // Number of observation and unknowns
        int nbrObservations = doubleDiffArrayList.size();
        int nbrUnknowns = 3;
        int nbrConstraint = 3;

        // Set approximate values for unknown parameters as coordinates of base station
        RealMatrix X_Rover = X_Base;

        // Initialization of the matrix
        RealMatrix l = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix f0 = new BlockRealMatrix(nbrObservations, 1);
        RealMatrix A = new BlockRealMatrix(nbrObservations, nbrUnknowns);

        RealMatrix dx = null;
        RealMatrix dl = null;
        RealMatrix P = null;
        BlockRealMatrix blockMatrixN = null;

        // Matrix for constrained calculation
        RealMatrix C = new Array2DRowRealMatrix(new double[][]{
                {1.0,0.0,0.0},
                {0.0,1.0,0.0},
                {0.0,0.0,1.0}
        });
        RealMatrix dX;
        RealMatrix Zcc = new BlockRealMatrix(nbrConstraint,nbrConstraint);

        // Stochastic model
        double sigma0 = 1.0;
        RealMatrix Kll = new BlockRealMatrix(nbrObservations, nbrObservations);

        // Compute iteration
        boolean cont = true;
        int nbr_iteration = 0;

        while (cont) {

            // Fill the matrix by going through each observation
            int idObs = 0;
            for (DoubleDiff doubleDiff : doubleDiffArrayList) {

                double doubleDiffVal = doubleDiff.getDoubleDiff();

                // Fill matrix l
                l.setEntry(idObs, 0, doubleDiffVal);

                // Fill matrix A by numerical derivation of the observation equation
                double f = computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);

                // Component x
                RealMatrix X_Rover_dx = X_Rover.copy();
                X_Rover_dx.addToEntry(0, 0, 1.0);
                double f_dx = computeObsEquation(X_Rover_dx, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 0, f_dx - f);

                // Component y
                RealMatrix X_Rover_dy = X_Rover.copy();
                X_Rover_dy.addToEntry(1, 0, 1.0);
                double f_dy = computeObsEquation(X_Rover_dy, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 1, f_dy - f);

                // Component z
                RealMatrix X_Rover_dz = X_Rover.copy();
                X_Rover_dz.addToEntry(2, 0, 1.0);
                double f_dz = computeObsEquation(X_Rover_dz, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                A.setEntry(idObs, 2, f_dz - f);

                // Fill f0
                f0.setEntry(idObs, 0, computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base));

                // Fill Kll
                Kll.setEntry(idObs,idObs,observationStandardDeviation*observationStandardDeviation);

                idObs += 1;
            }

            RealMatrix Qll = Kll.scalarMultiply(1 / (sigma0 * sigma0));
            P = new SingularValueDecomposition(Qll).getSolver().getInverse();
            dl = l.subtract(f0);

            RealMatrix t = constraintPoint.subtract(X_Rover);
            RealMatrix N = A.transpose().multiply(P).multiply(A);
            RealMatrix b = A.transpose().multiply(P).multiply(dl);

            blockMatrixN = new BlockRealMatrix(nbrUnknowns+nbrConstraint, nbrUnknowns+nbrConstraint);
            blockMatrixN.setSubMatrix(N.getData(), 0, 0);
            blockMatrixN.setSubMatrix(C.transpose().getData(), 0, N.getColumnDimension());
            blockMatrixN.setSubMatrix(C.getData(), N.getRowDimension(), 0);
            blockMatrixN.setSubMatrix(Zcc.getData(), N.getRowDimension(), N.getColumnDimension());

            BlockRealMatrix blockMatrixB = new BlockRealMatrix(nbrUnknowns+nbrConstraint,1);

            blockMatrixB.setSubMatrix(b.getData(),0,0);
            blockMatrixB.setSubMatrix(t.getData(),b.getRowDimension(),0);


            dX = new SingularValueDecomposition(blockMatrixN).getSolver().getInverse().multiply(blockMatrixB);

            dx = dX.getSubMatrix(0,nbrUnknowns-1,0,0);

            // Update unknown approximate vector
            X_Rover = X_Rover.add(dx);

            // Interruption criteria
            double maxNorm = 0.0;
            for (int i = 0; i < dx.getRowDimension(); i++) {
                double absValue = Math.abs(dx.getEntry(i, 0));
                if (absValue > maxNorm) {
                    maxNorm = absValue;
                }
            }

            if (maxNorm <= 1e-3) {
                cont = false;
            }
            if (nbr_iteration > 10){
                res.status = "no convergence, number of iteration > 10";
                cont = false;
            }

            // Update iteration for convergence of dx
            nbr_iteration++;
        }

        // Compute residuals v
        RealMatrix v = A.multiply(dx).subtract(dl);
        // Cofactor matrix of estimated parameters
        RealMatrix blockQxx = new SingularValueDecomposition(blockMatrixN).getSolver().getInverse();
        RealMatrix Qxx = blockQxx.getSubMatrix(0, nbrUnknowns-1, 0, nbrUnknowns-1);

        double threshold = 1e-12;
        for (int i = 0; i < Qxx.getRowDimension(); i++) {
            for (int j = 0; j < Qxx.getColumnDimension(); j++) {
                if (Math.abs(Qxx.getEntry(i, j)) < threshold) {
                    Qxx.setEntry(i, j, 0.0);
                }
            }
        }

        // Empirical standard deviation of unit weight
        double s0 = Math.sqrt(v.transpose().multiply(P).multiply(v).getEntry(0, 0) / (nbrObservations - nbrUnknowns + nbrConstraint));
        // Variance-covariance matrix of estimated parameters
        RealMatrix Kxx = Qxx.scalarMultiply(s0);


        //Indicators dilution of precision (DOPs)
        double GDOP = Math.sqrt(Qxx.getTrace());
        double PDOP = Math.sqrt(Qxx.getTrace());

        // Param ellipsoid Bessel
        double[] ellParam = ellBesselParam();
        double[] ellCoordinate = cart2ell(Constants.ELL_A_GRS80, ellParam[0], X_Rover);

        double lat0 = Math.toRadians(ellCoordinate[1]);
        double lon0 = Math.toRadians(ellCoordinate[0]);

        double sinLat0 = Math.sin(lat0);
        double cosLat0 = Math.cos(lat0);
        double sinLon0 = Math.sin(lon0);
        double cosLon0 = Math.cos(lon0);

        double[][] rotation = new double[][]{
                {-sinLat0 * cosLon0, -sinLat0 * sinLon0, cosLat0},
                {-sinLon0, cosLon0, 0},
                {cosLat0 * cosLon0, cosLat0 * sinLon0, sinLat0}
        };

        RealMatrix matrixR = new Array2DRowRealMatrix(rotation);

        RealMatrix matrixTopo = matrixR.multiply(Qxx.multiply(matrixR.transpose()));

        // HDOP and VDOP
        double HDOP = Math.sqrt(matrixTopo.getEntry(0, 0) + matrixTopo.getEntry(1, 1));
        double VDOP = Math.sqrt(matrixTopo.getEntry(2, 2));

        // Variance-covariance matrix of estimated parameters in topocentric system
        RealMatrix KxxTopo = matrixR.multiply(Kxx.multiply(matrixR.transpose()));
        double sigmaEast = Math.sqrt(KxxTopo.getEntry(1,1));
        double sigmaNorth = Math.sqrt(KxxTopo.getEntry(0,0));
        double sigmaHeight = Math.sqrt(KxxTopo.getEntry(2,2));

        // Save data in Result class
        res.s0 = s0;
        res.sigma0 = sigma0;
        res.X_Rover = X_Rover;
        res.Kxx = Kxx;
        res.GDOP = GDOP;
        res.PDOP = PDOP;
        res.VDOP = VDOP;
        res.HDOP = HDOP;
        res.Kll = Kll;
        res.doubleDiffArrayList = doubleDiffArrayList;
        res.v = v;
        res.nbrSatObserved = nbrSatObserved;
        res.nbrObservationsGps = nbrObservationGps;
        res.sigmaEast = sigmaEast;
        res.sigmaNorth = sigmaNorth;
        res.sigmaHeight = sigmaHeight;

        if(mUiFragmentComponent.isSendUDPChecked()){
            JSONObject json;
            try {
                json = res.toJSON("ResCons");
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            sendResultsUDP(json);
        }


        // Write results constraint in log file
        synchronized (mFileLockConstraint) {
            if (mFileWriterConstraint == null) {
                return;
            }
            try {
                String result = res.resultToString(true, mBaseStation);
                mFileWriterConstraint.write(result);
            } catch (IOException e) {
                logException(ERROR_WRITING_FILE, e);
            }
        }

    }


    private static class Results {
        public TimeE epoch;
        public int nbrObservationsGps;
        public String status;
        public double sigmaEast;

        public double sigmaNorth;
        public double sigmaHeight;
        private int nbrObs;

        private int nbrSatObserved;

        private int nbrUnknowns;

        private int nbrIteration;
        private double sigma0;

        private double s0;

        private double GDOP;
        private double PDOP;
        private double VDOP;
        private double HDOP;

        private RealMatrix Kll;

        private RealMatrix v;

        private RealMatrix X_Rover;

        private RealMatrix Kxx;

        private ArrayList<DoubleDiff> doubleDiffArrayList;


            // Method to convert the Results object to JSON
            public JSONObject toJSON(String type) throws JSONException {
                JSONObject json = new JSONObject();
                json.put("type",type);
                json.put("epoch", epoch != null ? epoch.toString() : JSONObject.NULL);
                json.put("nbrObservationsGps", nbrObservationsGps);
                json.put("status", status);
                json.put("sigmaEast", String.format(Locale.US, "%.3f", sigmaEast));
                json.put("sigmaNorth",String.format(Locale.US, "%.3f", sigmaNorth));
                json.put("sigmaHeight", String.format(Locale.US, "%.3f", sigmaHeight));
                json.put("nbrObs", nbrObs);
                json.put("nbrSatObserved", nbrSatObserved);
                json.put("nbrUnknowns", nbrUnknowns);
                json.put("nbrIteration", nbrIteration);
                json.put("sigma0", sigma0);
                json.put("s0", s0);
                json.put("GDOP", String.format(Locale.US, "%.1f", GDOP));
                json.put("PDOP", String.format(Locale.US, "%.1f", GDOP));
                json.put("VDOP", String.format(Locale.US, "%.1f", GDOP));
                json.put("HDOP", String.format(Locale.US, "%.1f", GDOP));

//                if (Kll != null) {
//                    json.put("Kll", matrixToJSONArray(Kll));
//                } else {
//                    json.put("Kll", JSONObject.NULL);
//                }

                if (v != null) {
                    json.put("v", matrixToJSONArray(v));
                } else {
                    json.put("v", JSONObject.NULL);
                }

                if (X_Rover != null) {
                    json.put("X_Rover", matrixToJSONArray(X_Rover));
                } else {
                    json.put("X_Rover", JSONObject.NULL);
                }

//                if (Kxx != null) {
//                    json.put("Kxx", matrixToJSONArray(Kxx));
//                } else {
//                    json.put("Kxx", JSONObject.NULL);
//                }

                if (doubleDiffArrayList != null) {
                    JSONArray doubleDiffArray = new JSONArray();
                    for (DoubleDiff doubleDiff : doubleDiffArrayList) {

                        doubleDiffArray.put(doubleDiff.toJSON());
                    }
                    json.put("doubleDiffArrayList", doubleDiffArray);
                } else {
                    json.put("doubleDiffArrayList", JSONObject.NULL);
                }

                return json;
            }

            // Helper method to convert RealMatrix to JSONArray
            private JSONArray matrixToJSONArray(RealMatrix matrix) throws JSONException {
                JSONArray jsonArray = new JSONArray();
                for (int i = 0; i < matrix.getRowDimension(); i++) {

                    if(matrix.getColumnDimension() ==1){
                        jsonArray.put(matrix.getEntry(i, 0));
                    } else {
                        JSONArray rowArray = new JSONArray();
                    for (int j = 0; j < matrix.getColumnDimension(); j++) {
                        rowArray.put(matrix.getEntry(i, j));
                    }
                    jsonArray.put(rowArray);}
                }
                return jsonArray;
        }

        public String resultToString(boolean isConstraint, BaseStation baseStation) {

            Date currentdate = new Date();

            double[] MN95 = CHTRS95toMN95(X_Rover);

            RealMatrix posBaseStation = baseStation.getStationaryAntenna();
            double xBaseStation = posBaseStation.getEntry(0,0);
            double yBaseStation = posBaseStation.getEntry(1,0);
            double zBaseStation = posBaseStation.getEntry(2,0);

            StringBuilder stringBuilder = new StringBuilder();

            StringBuilder doubleDiffObsStringBuilder = new StringBuilder();

            for (DoubleDiff DD : doubleDiffArrayList) {
                double sigma_l = Kll.getEntry(DD.iObs - 1, DD.iObs - 1);
                String str = String.format(Locale.US, "%4s %4s %4.0f %9.3f %9.3f %9.3f %13.3f %13.3f %13.3f %13.3f",
                        DD.getPRN1(), DD.getPRN2(), DD.getElevationPRN2(),
                        DD.getDoubleDiff(),
                        sigma_l,
                        v.getEntry(DD.iObs - 1, 0),
                        DD.zeroDiffBase.get(0), DD.zeroDiffBase.get(1),
                        DD.zeroDiffRover.get(0), DD.zeroDiffRover.get(1));
                doubleDiffObsStringBuilder.append(str).append("\n");
            }

            String computeType;
            if (isConstraint){
                computeType = "Constraint";
            }else{
                computeType = "Biber";
            }

            String header =
                    "---------------------------------------------------------------------------\n" +
                            "Differential Positioning (pseudo-distances) " + computeType + " / Elisa Borlat / HEIG-VD\n" +
                            currentdate + " \n" +
                            "Base station coordinates : " + xBaseStation + "   "+ yBaseStation + "   " + zBaseStation + "\n" +
                            "Signal : L1 (C1C) \n" +
                            "---------------------------------------------------------------------------" + "\n\n" ;

            stringBuilder.append(header);

            String resBuilder =

                    "Global stats :" + "\n" +
                    "=======================" + "\n" +
                    "Nbr observations : " + nbrObs + "\n" +
                    "Nbr unknowns : " + nbrUnknowns + "\n" +
                    "Nbr iteration : " + nbrIteration + " (10)\n" +
                    "s0/sigma0 : " + String.format(Locale.US, "%.2f", s0 / sigma0 ) + "\n\n" +

                    "Epoch : " + "\n" +
                    "=======" + "\n" +
                            epoch.getGpsTimeCalendar().toFormattedDateTime() + "\n" +

                    "Double differences observations (ePRN1 = " + doubleDiffArrayList.get(0).elevationPRN1 + ") : " + "\n" +
                    "=================================" + "\n" +
                    String.format("%-4s %-4s %-4s %9s %9s %9s %13s %13s %13s %13s\n", "PRN1", "PRN2", "ePRN2", "DD OBS", "E.T", "v", "ZD(BASE,PRN1)", "ZD(BASE,PRN2)", "ZD(ROVER,PRN1)", "ZD(ROVER,PRN2)") +
                    String.format("%-4s %-4s %-4s %9s %9s %9s %13s %13s %13s %13s\n", "", "", "[°]", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]") +

                    doubleDiffObsStringBuilder;

            stringBuilder.append(resBuilder).append("\n");

            String estimatedParam =
                    "Estimated parameters of the rover:" + "\n" +
                    "==================================" + "\n" +
                    String.format(Locale.US, "X WGS84 : %13.3f +/- %4.3f [m] \n",
                            X_Rover.getEntry(0, 0),
                            Math.sqrt(Math.abs(Kxx.getEntry(0, 0)))) +
                    String.format(Locale.US, "Y WGS84 : %13.3f +/- %4.3f [m] \n",
                            X_Rover.getEntry(1, 0),
                            Math.sqrt(Math.abs(Kxx.getEntry(1, 0)))) +
                    String.format(Locale.US, "Z WGS84 : %13.3f +/- %4.3f [m] \n",
                            X_Rover.getEntry(2, 0),
                            Math.sqrt(Math.abs(Kxx.getEntry(2, 0)))) + "\n";

            stringBuilder.append(estimatedParam);

            double[] MN95Rover = RealTimePositionCalculator.CHTRS95toMN95(X_Rover);

            // Derived parameters, coordinates MN95
            if (!isConstraint) {
                String derivativeParam =
                        "Derivative parameters:" + "\n" +
                                "========================================" + "\n" +
                                String.format(Locale.US, "E MN95 : %13.3f +/- %4.3f [m] \n",
                                        MN95Rover[0], sigmaEast) +
                                String.format(Locale.US, "N MN95 : %13.3f +/- %4.3f [m] \n",
                                        MN95Rover[1], sigmaNorth) +
                                String.format(Locale.US, "h Bessel : %11.3f +/- %4.3f [m] \n\n",
                                        MN95[2], sigmaHeight);

                stringBuilder.append(derivativeParam);
            }

            return stringBuilder.toString();

        }

    }

    private double computeObsEquation(RealMatrix X_Rover_init, DoubleDiff doubleDiff, Ephemeris.GpsNavMessageProto gpsNavMessageProto, TimeE timeOfRover, RealMatrix X_Base) {
        int prn1 = doubleDiff.getPRN1();
        int prn2 = doubleDiff.getPRN2();

        RealMatrix X_prn1 = getSVPosition(prn1, gpsNavMessageProto, timeOfRover);
        RealMatrix X_prn2 = getSVPosition(prn2, gpsNavMessageProto, timeOfRover);

        // Approx range
        double r0_1 = X_prn1.subtract(X_Rover_init).getFrobeniusNorm();
        double r0_2 = X_prn2.subtract(X_Rover_init).getFrobeniusNorm();

        // t_emit
        double mjd_sec_of_day_emit_1 = timeOfRover.getMjd().doublePart - r0_1 / Constants.SPEED_OF_LIGHT;
        double mjd_sec_of_day_emit_2 = timeOfRover.getMjd().doublePart - r0_2 / Constants.SPEED_OF_LIGHT;

        TimeE.FormatTime mjd_emit_1 = new TimeE.FormatTime(timeOfRover.getMjd().integerPart, mjd_sec_of_day_emit_1);
        TimeE.FormatTime mjd_emit_2 = new TimeE.FormatTime(timeOfRover.getMjd().integerPart, mjd_sec_of_day_emit_2);

        TimeE timeOfEmit1 = new TimeE(mjd_emit_1);
        TimeE timeOfEmit2 = new TimeE(mjd_emit_2);

        // Satellite position at t_emit
        RealMatrix X_prn1_emit = getSVPosition(prn1, gpsNavMessageProto, timeOfEmit1);
        RealMatrix X_prn2_emit = getSVPosition(prn2, gpsNavMessageProto, timeOfEmit2);

        // Approx range
        r0_1 = X_prn1_emit.subtract(X_Rover_init).getFrobeniusNorm();
        r0_2 = X_prn2_emit.subtract(X_Rover_init).getFrobeniusNorm();

        // Earth rotation during propagation
        double alpha_1 = (r0_1 / Constants.SPEED_OF_LIGHT) * Constants.EARTH_ANGULAR_VELOCITY;
        double alpha_2 = (r0_2 / Constants.SPEED_OF_LIGHT) * Constants.EARTH_ANGULAR_VELOCITY;

        RealMatrix R_1 = new Array2DRowRealMatrix(new double[][]{
                {Math.cos(alpha_1), Math.sin(alpha_1), 0},
                {-Math.sin(alpha_1), Math.cos(alpha_1), 0},
                {0, 0, 1.0}
        });

        RealMatrix R_2 = new Array2DRowRealMatrix(new double[][]{
                {Math.cos(alpha_2), Math.sin(alpha_2), 0},
                {-Math.sin(alpha_2), Math.cos(alpha_2), 0},
                {0, 0, 1.0}
        });
        // Sagnac effect correction
        X_prn1_emit = R_1.multiply(X_prn1_emit);
        X_prn2_emit = R_2.multiply(X_prn2_emit);


        // Observation equation
        double pr_base_1 = X_prn1_emit.subtract(X_Base).getFrobeniusNorm();
        double pr_base_2 = X_prn2_emit.subtract(X_Base).getFrobeniusNorm();
        double pr_rover_1 = X_prn1_emit.subtract(X_Rover_init).getFrobeniusNorm();
        double pr_rover_2 = X_prn2_emit.subtract(X_Rover_init).getFrobeniusNorm();

        return (pr_base_1 - pr_rover_1) - (pr_base_2 - pr_rover_2);
    }

    private double computePseudorange(GnssMeasurement mes, GnssClock gnssClock) {

        /**maintaining constant the 'FullBiasNanos' instead of using the instantaneous value. This avoids the 256 ns
         jumps each 3 seconds that create a code-phase divergence due to the clock.*/
        if (!setClockBias) {
            fullBiasNanos = gnssClock.getFullBiasNanos();
            BiasNanos = gnssClock.getBiasNanos();
            setClockBias = true;
            System.out.println("Temp| setClockBias:"+gnssClock.getFullBiasNanos());
        }


        double tTx = mes.getReceivedSvTimeNanos();
        double tRxGNSS = gnssClock.getTimeNanos() + mes.getTimeOffsetNanos() - (fullBiasNanos + BiasNanos);
        double tRx = tRxGNSS % Constants.NUMBER_NANO_SECONDS_WEEK;

        return (tRx - tTx) * 1e-9 * Constants.SPEED_OF_LIGHT;
    }

    static class DoubleDiff {

        public int iObs;

        public final Integer PRN1;

        public final Integer PRN2;

        public Float elevationPRN1;

        public Float elevationPRN2;

        public final Map<Integer, Double> zeroDiffBase = new HashMap<>();

        public final Map<Integer, Double> zeroDiffRover = new HashMap<>();

        public double doubleDiff;

        public DoubleDiff(Integer PRN1, Integer PRN2) {
            this.PRN1 = PRN1;
            this.PRN2 = PRN2;
        }

        public Integer getPRN1() {
            return PRN1;
        }

        public Integer getPRN2() {
            return PRN2;
        }

        public double getDoubleDiff() {
            return doubleDiff;
        }

        public Float getElevationPRN2() {
            return elevationPRN2;
        }

        public JSONObject toJSON() {
            JSONObject jsonObject = new JSONObject();
            try {
                jsonObject.put("iObs", iObs);
                jsonObject.put("PRN1", PRN1);
                jsonObject.put("PRN2", PRN2);
                jsonObject.put("elevationPRN1", elevationPRN1);
                jsonObject.put("elevationPRN2", elevationPRN2);
                jsonObject.put("zeroDiffBase", zeroDiffBase);
                jsonObject.put("zeroDiffRover", zeroDiffRover);
                jsonObject.put("doubleDiff", doubleDiff);
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }


            return jsonObject;
        }
    }

    private boolean hasEphemeris(Integer value) {

        // Check Ephemeris is available for sv and refSv less than 2 hours ago
        System.currentTimeMillis();
        GpsTime systemGpsTime = GpsTime.now();
        Pair<Integer, Integer> systemGpsWeekSecond = systemGpsTime.getGpsWeekSecond();
        Integer systemSecond = systemGpsWeekSecond.second;

        Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

        if (gpsNavMessageProto != null) {

            for (Ephemeris.GpsEphemerisProto eph : gpsNavMessageProto.ephemerids) {

                if ((eph.prn == value) &&
                        (eph.toe - systemSecond) < 7200) { // Time max since last ephemeris update : 2 hours
                    return true;
                }
            }
        }
        return false;
    }

    public RealMatrix getSVPosition(int prn, Ephemeris.GpsNavMessageProto mHardwareGpsNavMessageProto, TimeE timeE) {

        StringBuilder log = new StringBuilder(); // Create a StringBuilder for logging
        RealMatrix X_WGS84 = new BlockRealMatrix(3, 1);
        X_WGS84.setColumn(0, new double[]{0.0, 0.0, 0.0});

        // Check if PRN exists in ephemeris data
        Ephemeris.GpsEphemerisProto prnEphemeris;
        for (Ephemeris.GpsEphemerisProto eph : mHardwareGpsNavMessageProto.ephemerids) {
            if (eph.prn == prn) {
                // Get ephemeris data for the current SV
                prnEphemeris = eph;

                // Delta temps
                double dt = timeE.getTow() - prnEphemeris.toc;

                // Handle week wraparound
                if (dt > 302400.0) {
                    dt -= 604800.0;
                } else if (dt < -302400.0) {
                    dt += 604800.0;
                }

                // Mean anomaly
                double a = prnEphemeris.rootOfA * prnEphemeris.rootOfA;
                double n0 = Math.sqrt(com.example.pseudorange.Constants.GM / (a * a * a));
                double n = n0 + prnEphemeris.deltaN;
                double mK = prnEphemeris.m0 + n * dt;

                // Eccentric anomaly
                double eK = mK;
                while (Math.abs(eK - (mK + prnEphemeris.e * Math.sin(eK))) > 1e-12) {
                    eK = mK + prnEphemeris.e * Math.sin(eK);
                }

                // True anomaly
                double vK = Math.atan2(Math.sqrt(1 - prnEphemeris.e * prnEphemeris.e) * Math.sin(eK),
                        Math.cos(eK) - prnEphemeris.e);

                // Harmonic terms
                double hc = Math.cos(2.0 * (prnEphemeris.omega + vK));
                double hs = Math.sin(2.0 * (prnEphemeris.omega + vK));

                // Argument of latitude
                double uK = vK + prnEphemeris.omega + prnEphemeris.cuc * hc + prnEphemeris.cus * hs;

                // Radial distance
                double rK = a * (1 - prnEphemeris.e * Math.cos(eK)) + prnEphemeris.crc * hc + prnEphemeris.crs * hs;

                // Coordinates in orbital plane
                double x = rK * Math.cos(uK);
                double y = rK * Math.sin(uK);

                // Inclination
                double iK = prnEphemeris.i0 + prnEphemeris.iDot * dt + prnEphemeris.cic * hc + prnEphemeris.cis * hs;

                // Velocity part
                double eKDot = n / (1 - prnEphemeris.e * Math.cos(eK));
                double vKDot = Math.sin(eK) * eKDot * (1 + prnEphemeris.e * Math.cos(vK)) /
                        ((1 - Math.cos(eK) * prnEphemeris.e) * Math.sin(vK));

                double uKDot = vKDot + 2 * (prnEphemeris.cus * hc - prnEphemeris.cuc * hs) * vKDot;

                double rKDot = a * prnEphemeris.e * Math.sin(eK) * eKDot +
                        2 * ((prnEphemeris.crs * hc - prnEphemeris.crc * hs) * vKDot);

                double iKDot = prnEphemeris.iDot + 2 * ((prnEphemeris.cis * hc - prnEphemeris.cic * hs) * vKDot);

                double lambda_k_dot = prnEphemeris.omegaDot - com.example.pseudorange.Constants.w_e;

                double x_dot = rKDot * Math.cos(uK) - rK * Math.sin(uK) * uKDot;
                double y_dot = rKDot * Math.sin(uK) + rK * Math.cos(uK) * uKDot;

                // Longitude of ascending node
                double lambdaK = prnEphemeris.omega0 + (prnEphemeris.omegaDot - com.example.pseudorange.Constants.w_e) * dt - com.example.pseudorange.Constants.w_e * prnEphemeris.toe;

                // Coordinate transformation from orbital plane to WGS84
                double cl = Math.cos(lambdaK);
                double sl = Math.sin(lambdaK);
                double ci = Math.cos(iK);
                double si = Math.sin(iK);

                double x_WGS84 = x * cl - y * ci * sl;
                double y_WGS84 = x * sl + y * ci * cl;
                double z_WGS84 = y * si;

                X_WGS84.setColumn(0, new double[]{x_WGS84, y_WGS84, z_WGS84});


                double V_WGS84_x = x_dot * cl - y_dot * ci * sl + y * si * sl * iKDot - y_WGS84 * lambda_k_dot;
                double V_WGS84_y = x_dot * sl + y_dot * ci * cl - y * si * cl * iKDot + x_WGS84 * lambda_k_dot;
                double V_WGS84_z = y_dot * si + y * ci * iKDot;

                double[] V_WGS84 = new double[]{V_WGS84_x, V_WGS84_y, V_WGS84_z};

                log.append("**Ephemeris Data for PRN ").append(prn).append("**\n");
                log.append("PRN: ").append(prnEphemeris.prn).append("\n");
                log.append("toe: ").append(prnEphemeris.toe).append("\n");
                log.append("Week: ").append(prnEphemeris.week).append("\n");
                log.append("**Calculation position WGS84**\n");
                log.append("Mean anomaly mK: ").append(mK).append("\n");
                log.append("Eccentric anomaly eK: ").append(eK).append("\n");
                log.append("True anomaly vK: ").append(vK).append("\n");
                log.append("Argument of latitude uK: ").append(uK).append("\n");
                log.append("Radial distance rK: ").append(rK).append("\n");
                log.append("Coordinates in orbital plane x : ").append(x).append("\n");
                log.append("Coordinates in orbital plane y : ").append(y).append("\n");
                log.append("Inclination iK: ").append(iK).append("\n");
                log.append("Longitude of ascending node lambdaK: ").append(lambdaK).append("\n");

            }
        }
        return X_WGS84;
    }

    public static double[] CHTRS95toMN95(RealMatrix X) {

        // Transformation CHTRS95 => CH1903+
        RealMatrix t = new Array2DRowRealMatrix(new double[][]{
                {-674.374},
                {-15.056},
                {-405.346}
        });
        t = X.add(t);

        // Param ellipsoid Bessel
        double[] ellParam = ellBesselParam();
        double e = ellParam[0];

        // Ellipsoidal coordinates CH1903+
        double[] result = cart2ell(Constants.ELL_A_Bessel, e, t);


        return ell2EN(result[0], result[1], result[2]);
    }

    public static double[] MN95toCHTRS(double east, double north, double ellHeight) {

        double[] ellCH1903p = EN2ell(east,north);
        double[] X_CH1903p = ell2cart(ellCH1903p[0],ellCH1903p[1],ellHeight);
        double x_CHTRS = X_CH1903p[0] + 674.374;
        double y_CHTRS = X_CH1903p[1] + 15.056;
        double z_CHTRS = X_CH1903p[2] + 405.346;

        return new double[]{x_CHTRS,y_CHTRS,z_CHTRS};
    }

    public void startNewLog(){
        synchronized (mFileLock){
            mUiFragmentComponent.resetDetailedCounter();
            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss", Locale.US);
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
        synchronized (mFileLockConstraint){
            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss", Locale.US);
            Date now = new Date();
            String fileName = String.format("%s_%s.txt", FILE_PREFIX_CONSTRAINT, formatter.format(now));
            File currentFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), fileName);
            String currentFilePath = currentFile.getAbsolutePath();

            BufferedWriter currentFileWriter;
            try {
                currentFileWriter = new BufferedWriter(new FileWriter(currentFile));
            } catch (IOException e) {
                logException("Could not open file: " + currentFilePath, e);
                return;
            }

            if (mFileWriterConstraint != null) {
                try {
                    mFileWriterConstraint.close();
                } catch (IOException e) {
                    logException("Unable to close all file streams.", e);
                    return;
                }
            }

            mFileConstraint = currentFile;
            mFileWriterConstraint = currentFileWriter;
            Toast.makeText(mContext, "File opened: " + currentFilePath, Toast.LENGTH_SHORT).show();
        }
    }



    public synchronized void setUiFragmentComponent(HomeFragment.HomeUIFragmentComponent value) {
        mUiFragmentComponent = value;
    }

    public synchronized void setStatusUiFragmentComponent(StatusFragment.StatusUIFragmentComponent value) {
        mStatusUIFragmentComponent = value;
    }


    public static double[] cart2ell(double a, double e, RealMatrix t) {
        double[] result = new double[3];
        double x = t.getEntry(0, 0);
        double y = t.getEntry(1, 0);
        double z = t.getEntry(2, 0);

        // Ellipsoidal longitude radians
        double lon_rad = Math.atan2(y, x);

        // Ellipsoidal longitude degrees
        double lon_deg = lon_rad * 180.0 / Math.PI;

        // Initialisation ellipsoidal height and radius of curvature of the normal section
        double hi = 0;
        double him1 = 1;
        double RNi = 1;
        double lat_radi = 0;

        // Iterative loop to calculate latitude and ellipsoidal height
        while (Math.abs(hi - him1) > 0.000001) {
            him1 = hi;

            double sqrt = Math.sqrt(x * x + y * y);
            lat_radi = Math.atan2(z, sqrt * (1 - (RNi / (RNi + hi)) * e * e));
            RNi = a / Math.sqrt(1 - e * e * Math.sin(lat_radi) * Math.sin(lat_radi));
            hi = sqrt / Math.cos(lat_radi) - RNi;
        }

        // Latitude in degrees and ellipsoidal height
        double lat_deg = lat_radi * 180.0 / Math.PI;
        double h = hi;

        result[0] = lon_deg;
        result[1] = lat_deg;
        result[2] = h;

        return result;
    }

    public static double[] ell2cart(double lon_deg, double lat_deg, double h) {
        double lon_rad = Math.toRadians(lon_deg);
        double lat_rad = Math.toRadians(lat_deg);

        double[] paramBessel = ellBesselParam();
        double RN = Constants.ELL_A_Bessel/Math.sqrt(1.0-Math.pow(paramBessel[0],2.0)*
                Math.pow(Math.sin(lat_rad),2.0));

        double x = (RN + h) * Math.cos(lat_rad) * Math.cos(lon_rad);
        double y = (RN + h) * Math.cos(lat_rad) * Math.sin(lon_rad);
        double z = (RN * (1 - paramBessel[0] * paramBessel[0]) + h) * Math.sin(lat_rad);
        return new double[]{x,y,z};
    }



    public static double[] ell2EN(double lon_deg, double lat_deg, double height) {

            double lon = lon_deg * Math.PI / 180.0;
            double lat = lat_deg * Math.PI / 180.0;

            double lon0 = (7.0 + 26.0 / 60.0 + 22.50 / 3600.0) * Math.PI / 180.0;
            double alpha = 1.0007291384304;
            double k = 1.0030714396280;
            double lat_sph_0 = (46.0 + 54.0 / 60.0 + 27.83324846 / 3600.0) * Math.PI / 180.0;
            double R_sph = 6378815.90365;
            double E0, N0;


            E0 = 2600000.000;
            N0 = 1200000.000;


            // ellipsoid -> normal sphere
            double lon_sph = alpha * (lon - lon0);
            double lat_sph = 2 * Math.atan(
                    k * Math.pow(Math.tan(Math.PI / 4 + lat / 2.0), alpha) * Math.pow((1 - ellBesselParam()[0] * Math.sin(lat)) / (1 + ellBesselParam()[0] * Math.sin(lat)),
                            alpha * ellBesselParam()[0] / 2.0))
                    - Math.PI / 2.0;

            // normal sphere -> oblique sphere
            double lon_sph_t = Math.atan(Math.sin(lon_sph) / (Math.sin(lat_sph_0) * Math.tan(lat_sph)
                    + Math.cos(lat_sph_0) * Math.cos(lon_sph)));
            double lat_sph_t = Math.asin(Math.cos(lat_sph_0) * Math.sin(lat_sph)
                    - Math.sin(lat_sph_0) * Math.cos(lat_sph) * Math.cos(lon_sph));

            // oblique sphere -> plane
            double E = E0 + R_sph * lon_sph_t;
            double N = N0 + R_sph * Math.log(Math.tan(Math.PI / 4.0 + lat_sph_t / 2.0));

            return new double[]{E,N,height};
    }

    public static double[] EN2ell(double east, double north) {

        double[] besselParam = ellBesselParam();

        double lon0 = (7.0 + 26.0 / 60.0 + 22.50 / 3600.0) * Math.PI / 180.0;
        double alpha = 1.0007291384304;
        double k = 1.0030714396280;
        double lat_sph_0 = (46.0 + 54.0 / 60.0 + 27.83324846 / 3600.0) * Math.PI / 180.0;
        double R_sph = 6378815.90365;

        double east0 = 2600000.000;
        double north0 = 1200000.000;

        double lon_sph_t = (east-east0)/R_sph;
        double lat_sph_t = 2.0*Math.atan(Math.exp((north-north0)/R_sph))-Math.PI/2.0;

        double lon_sph = Math.atan(Math.sin(lon_sph_t)/(Math.cos(lat_sph_0)*Math.cos(lon_sph_t) - Math.sin(lat_sph_0)*Math.tan(lat_sph_t)));
        double lat_sph = Math.asin(Math.cos(lat_sph_0)*Math.sin(lat_sph_t)+ Math.sin(lat_sph_0)*Math.cos(lat_sph_t)*Math.cos(lon_sph_t));

        double lon = lon0 + 1.0/alpha*lon_sph;
        double lati = lat_sph+1.0;
        double lat = lat_sph;
        while(Math.abs(lati-lat)>0.000000000001){
            lati=lat;
            lat = 2.0*Math.atan(
                    Math.pow(Math.tan(Math.PI/4.0+lat_sph/2.0),1.0/alpha) *
                            Math.pow(k, -1.0/alpha) *
                            Math.pow((1.0+besselParam[0]*Math.sin(lati))/
                                    (1.0-besselParam[0]*Math.sin(lati)),
                                    besselParam[0]/2.0))
                    - Math.PI/2.0;
        }
        double lon_deg = Math.toDegrees(lon);
        double lat_deg = Math.toDegrees(lat);

        return new double[]{lon_deg,lat_deg};
    }

    private static double[] ellBesselParam() {
        double a = Constants.ELL_A_Bessel;
        double f = Constants.ELL_F_Bessel;
        double b = a - a * f;
        double e = Math.sqrt(a * a - b * b) / a;
        return new double[]{e, b};
    }

    private void logException(String errorMessage, Exception e) {
        Log.e(MeasurementProvider.TAG + TAG, errorMessage, e);
        Toast.makeText(mContext, errorMessage, Toast.LENGTH_LONG).show();
    }

    public void send() {
        mUiFragmentComponent.resetDetailedCounter();

        if (mFile != null) {
            if (mFileWriter != null) {
                try {
                    mFileWriter.flush();
                    mFileWriter.close();
                    mFileWriter = null;
                } catch (IOException e) {
                    logException("Unable to close mFileWriter streams.", e);
                } finally {
                    mFileWriter = null;
                }
            }
        }

        if (mFileConstraint == null) {
            if (mFileWriterConstraint != null) {
                try {
                    mFileWriterConstraint.flush();
                    mFileWriterConstraint.close();
                    mFileWriterConstraint = null;
                } catch (IOException e) {
                    logException("Unable to close mFileWriterConstraint streams.", e);
                } finally {
                    mFileWriterConstraint = null;
                }
            }
        }
    }

    private void sendResultsUDP(JSONObject json){

        new Thread(() -> {
            String ipAddress = "192.168.206.161"; // Adresse IP de destination
            int port = 5005;                // Port de destination

            UDPSender sender = new UDPSender(ipAddress, port);

            sender.sendJSONObject(json);
            sender.close();
        }).start();
    }
}
