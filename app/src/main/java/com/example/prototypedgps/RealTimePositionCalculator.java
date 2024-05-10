package com.example.prototypedgps;

import android.location.GnssClock;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.location.Location;
import android.location.cts.nano.Ephemeris;
import android.os.Bundle;
import android.os.Environment;
import android.util.Pair;

import com.example.pseudorange.EphemerisManager;
import com.example.pseudorange.GpsTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.SingularValueDecomposition;

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

        Boolean setClockBias = false;
        private static  double fullBiasNanos = 1.0e-9,BiasNanos = 1.0e-9;


    private final BaseStation mBaseStation;

        private final EphemerisManager mEphemerisManager;

        private GnssStatus mGnssStatus;

        private final ArrayList<Results> allResults = new ArrayList<>();


        public RealTimePositionCalculator(BaseStation baseStation, EphemerisManager ephemerisManager) {
            this.mBaseStation = baseStation;
            this.mEphemerisManager = ephemerisManager;

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

            // Wait 1 second for the last RTCM messages to be decoded
            try {
                Thread.sleep(1000); // 1000 millisecond = 1 second
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
            if(!mBaseStation.isStationaryAntennaDecoded()){
                System.out.println("computePos| Position of base station not decoded");
                return;
            }

            System.out.println("\n--------------------------------------");
            System.out.println("Code differential estimation");
            System.out.println("--------------------------------------\n");

            // Epoch of position computation
            GnssClock gnssClock = event.getClock();

            // Check that the receiver have estimate GPS time
            if (!gnssClock.hasFullBiasNanos()) {
                System.out.println("computeDGPSSingleEpoch| FullBiasNanos is not decoded, exit positioning calculation");
                return;
            }

            double gpsTimeNanos = gnssClock.getTimeNanos() - (gnssClock.getFullBiasNanos() + gnssClock.getBiasNanos());
            TimeE timeOfRover = new TimeE(gpsTimeNanos/1e6);
            System.out.println("computeDGPSSingleEpoch| gnssClock: " + timeOfRover.getGpsTimeMilliseconds());

            // Base station
            ArrayList<Observations> observationsArrayList = mBaseStationReceiver.getEpochsArrayList();
            int size = observationsArrayList.size();

            if (size < 4) {
                System.out.println("computeDGPSSingleEpoch| Number of decoded base station observations < 4 , exit positioning calculation");
                return;
            }

            // Search base station closest observation
            double deltaMin = 100000;
            double timeNear = 0;
            double timeBefor = 0;

            Observations baseStationBeforNearObservations = null;
            Observations baseStationNearObservations = null;

            for (int i = size - 1; i >= 0; i--) {

                Observations iObservation = observationsArrayList.get(i);
                Time iTime = iObservation.getRefTime();
                double iGpsTime = iTime.getMsecGpsTime();
                System.out.println("debug| currentGpsTime: " + iTime.getMsecGpsTime());
                double iDelta = Math.abs(gpsTimeNanos / 1e6 - iGpsTime);

                if (iDelta > deltaMin) {

                    baseStationNearObservations = observationsArrayList.get(i + 1);
                    timeNear = baseStationNearObservations.getRefTime().getMsecGpsTime()/1e3;
                    baseStationBeforNearObservations = observationsArrayList.get(i);
                    timeBefor = baseStationBeforNearObservations.getRefTime().getMsecGpsTime()/1e3;
                    System.out.println("debug| closest: " +timeNear);
                    System.out.println("debug| before closest : " +timeBefor);

                    break; // Get out of the loop when iDelta > deltaMin
                } else {
                    deltaMin = iDelta;
                }
            }

            if (deltaMin > 1000.0) {
                return;
            }

            System.out.printf("computeDGPSSingleEpoch| delta (msec):  %.0f\n", deltaMin);

            // Initialization lists of available svId
            ArrayList<Integer> svIdRover = new ArrayList<>();
            ArrayList<Integer> svIdBaseStation = new ArrayList<>();
            ArrayList<Integer> commonSvId = new ArrayList<>();

            for (GnssMeasurement mes : event.getMeasurements()) {

                if (mes.getConstellationType() == GnssStatus.CONSTELLATION_GPS
                        && mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                        && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0 ) {
                    svIdRover.add(mes.getSvid());
                }
            }

            for (ObservationSet obs : baseStationNearObservations.getObsSet()) {
                svIdBaseStation.add(obs.getSatID());
            }

            // Common available svId
            for (Integer value : svIdRover) {

                if (svIdBaseStation.contains(value)
                        && hasEphemeris(value)) {
                    commonSvId.add(value);
                }
            }

            if(commonSvId.size()<4){
                System.out.println("computeDGPSSingleEpoch| Number of common sat < 4 , exit positioning calculation");
                return;
            }
            System.out.println("computeDGPSSingleEpoch| common svId : " + commonSvId);

            ArrayList<DoubleDiff> doubleDiffArrayList = new ArrayList<>();

            int refSv = 0;
            float elevationRefSv = 0;
            for (Integer sv : commonSvId) {

                for(int i = 0; i< mGnssStatus.getSatelliteCount(); i++){
                    if (mGnssStatus.getConstellationType(i) == GnssStatus.CONSTELLATION_GPS
                            && mGnssStatus.getSvid(i) == sv
                            && mGnssStatus.getElevationDegrees(i) > elevationRefSv){
                        refSv = mGnssStatus.getSvid(i);
                        elevationRefSv = mGnssStatus.getElevationDegrees(i);
                    };
                }

            }

//        int refSv = commonSvId.get(0);

            ObservationSet observationSetRefSv = null;
            ObservationSet observationSetRefSvBefor = null;

            int idObs = 0;
            for (Integer sv : commonSvId) {

                double pseudorangeSvBaseStation = 0;
                double pseudorangeSvBaseStationBefor = 0;
                double pseudorangeSvRover = 0;
                double pseudorangeSvRefRover = 0;

                for (GnssMeasurement mes : event.getMeasurements()) {
                    if (mes.getConstellationType() == GnssStatus.CONSTELLATION_GPS
                            && hasEphemeris(mes.getSvid())
                            && mes.getCn0DbHz() >= C_TO_N0_THRESHOLD_DB_HZ
                            && (mes.getState() & (1L << TOW_DECODED_MEASUREMENT_STATE_BIT)) != 0 ) {
                        if (mes.getSvid() == sv) {
                            pseudorangeSvRover = computePseudorange(mes, gnssClock);
                        }
                        if (mes.getSvid() == refSv) {
                            pseudorangeSvRefRover = computePseudorange(mes, gnssClock);
                        }
                    }

                }

                for (ObservationSet obs : baseStationNearObservations.getObsSet()) {
                    if (obs.getSatID() == sv) {
                        pseudorangeSvBaseStation = obs.getPseudorange(0);
                    }
                    if (obs.getSatID() == refSv) {
                        observationSetRefSv = obs;
                    }
                }

                for (ObservationSet obs : baseStationBeforNearObservations.getObsSet()){
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
                    double doppler = (pseudorangeSvBaseStation-pseudorangeSvBaseStationBefor)/(timeNear-timeBefor);
                    double pseudorangeSvBaseStationSync = pseudorangeSvBaseStation + doppler * deltaMin/1000 * Constants.SPEED_OF_LIGHT/Constants.FL1 ;
                    double simpleDiffSv = pseudorangeSvBaseStationSync - pseudorangeSvRover;
                    System.out.printf("computeDGPSSingleEpoch| pseudorange sat %s [Base, Rover]: [%f,%f]\n", sv, pseudorangeSvBaseStationSync, pseudorangeSvRover);
                    // SvRef
                    double dopplerRef = (observationSetRefSv.getPseudorange(0)-observationSetRefSvBefor.getPseudorange(0))/(timeNear-timeBefor);

                    System.out.println("computeDGPSSingleEpoch| timeNear-timeBefor : " + (timeNear-timeBefor));
                    double pseudorangeRefSvBaseStationSyncro = observationSetRefSv.getPseudorange(0) + dopplerRef * deltaMin/1000 * Constants.SPEED_OF_LIGHT/Constants.FL1 ;
                    double simpleDiffRef = pseudorangeRefSvBaseStationSyncro - pseudorangeSvRefRover;
                    System.out.printf("computeDGPSSingleEpoch| pseudorange sat ref %s [Base, Rover]: [%f,%f]\n", refSv, pseudorangeRefSvBaseStationSyncro, pseudorangeSvRefRover);

                    double doubleDiff = simpleDiffRef - simpleDiffSv;
                    System.out.println("computeDGPSSingleEpoch| double difference : " + doubleDiff);

                    DoubleDiff doubleDiffObs = new DoubleDiff(refSv, sv);
                    doubleDiffObs.zeroDiffBase.put(0, pseudorangeRefSvBaseStationSyncro);
                    doubleDiffObs.zeroDiffBase.put(1,pseudorangeSvBaseStationSync);
                    doubleDiffObs.zeroDiffRover.put(0,pseudorangeSvRefRover);
                    doubleDiffObs.zeroDiffRover.put(1,pseudorangeSvRover);
                    doubleDiffObs.doubleDiff = doubleDiff;
                    doubleDiffObs.elevationPRN1 = elevationRefSv;
                    for(int i = 0; i< mGnssStatus.getSatelliteCount(); i++){
                        if (mGnssStatus.getConstellationType(i) == GnssStatus.CONSTELLATION_GPS
                                && mGnssStatus.getSvid(i) == sv){
                            doubleDiffObs.elevationPRN2 = mGnssStatus.getElevationDegrees(sv);
                        }
                    }

                    idObs += 1;
                    doubleDiffObs.iObs = idObs;
                    doubleDiffArrayList.add(doubleDiffObs);
                }
            }

            if (doubleDiffArrayList.size() < 4) {
                return;
            }
            computePos(doubleDiffArrayList, timeOfRover);

        }


        private void computePos(ArrayList<DoubleDiff> doubleDiffArrayList, TimeE timeOfRover) {

            Results res = new Results();

            int nbrObservations = doubleDiffArrayList.size();
            int nbrUnknowns = 3;


            RealMatrix X_Base = mBaseStation.getStationaryAntenna();
            RealMatrix X_Rover = X_Base;

            // Ephemeris
            Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

            // Initialization of the matrix
            RealMatrix l = new BlockRealMatrix(nbrObservations, 1);
            RealMatrix f0 = new BlockRealMatrix(nbrObservations, 1);
            RealMatrix Kll = new BlockRealMatrix(nbrObservations,nbrObservations);
            RealMatrix A = new BlockRealMatrix(nbrObservations,nbrUnknowns);

            RealMatrix dx = null;
            RealMatrix dl = null;
            double sigma0 = 1.0;
            RealMatrix Qll;
            RealMatrix P = null;
            RealMatrix Qxx;
            RealMatrix v;
            double s0;
            double GDOP;
            RealMatrix Kxx;

            // Compute a iteration
            boolean cont = true;
            boolean first_convergence = false;
            int nbr_iter = 0;

            while (cont) {

                nbr_iter++;

                Iteration currentIteration = new Iteration();
                currentIteration.id = nbr_iter;

                // Fill the matrix by going through each observation
                int idObs = 0;
                for(DoubleDiff doubleDiff: doubleDiffArrayList){

                    double doubleDiffVal = doubleDiff.getDoubleDiff();

                    // Fill matrix l
                    l.setEntry(idObs, 0, doubleDiffVal);

                    // Fill matrix A by numerical derivation of the observation equation
                    double f = computeObsEquation(X_Rover, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);

                    // Component x
                    RealMatrix derivatives_x = X_Rover.copy();
                    derivatives_x.addToEntry(0, 0, 1.0);
                    double f_dx = computeObsEquation(derivatives_x, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 0, f_dx - f);

                    // Component y
                    RealMatrix derivatives_y = X_Rover.copy();
                    derivatives_y.addToEntry(1, 0, 1.0);
                    double f_dy = computeObsEquation(derivatives_y, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 1, f_dy - f);

                    // Component z
                    RealMatrix derivatives_z = X_Rover.copy();
                    derivatives_z.addToEntry(2, 0, 1.0);
                    double f_dz = computeObsEquation(derivatives_z, doubleDiff, gpsNavMessageProto, timeOfRover, X_Base);
                    A.setEntry(idObs, 2, f_dz - f);

                    // Fill f0
                    f0.setEntry(idObs, 0, computeObsEquation(X_Rover,doubleDiff, gpsNavMessageProto, timeOfRover, X_Base));

                    // Fill K0
                    if (!first_convergence) {
                        Kll.setEntry(idObs, idObs, 1.0);
                    }
                    idObs += 1 ;

                    currentIteration.A = A;

                    res.iterations.put(currentIteration.id, currentIteration);
                }


                dl = l.subtract(f0);
                Qll = Kll.scalarMultiply(1 / (sigma0 * sigma0));
                P = new SingularValueDecomposition(Qll).getSolver().getInverse();

                dx = new SingularValueDecomposition(A.transpose().multiply(P).multiply(A)).getSolver().getInverse().multiply(A.transpose()).multiply(P).multiply(dl);

                // Update unknowns vector
                X_Rover = X_Rover.add(dx);

                if (first_convergence && dx.getFrobeniusNorm() <= 1e-3) {
                    cont = false;
                    System.out.println("computePos| Convergence");
                }

                if (!first_convergence && dx.getFrobeniusNorm() <= 1e-3) {
                    first_convergence = true;
                }

                if (nbr_iter >= 10) {
                    cont = false;
                    System.out.println("computePos| Number of iteration exceeded max iteration (10)");
                }
            }

            System.out.println("computePos| X_Rover: "+ X_Rover);

            v = A.multiply(dx).subtract(dl);
            double vTPv = v.transpose().multiply(P).multiply(v).getEntry(0,0);
            s0 = Math.sqrt(vTPv / (nbrObservations - nbrUnknowns));
            Qxx = new SingularValueDecomposition(A.transpose().multiply(P).multiply(A)).getSolver().getInverse();
            GDOP = Math.sqrt(Qxx.getTrace());
            Kxx = Qxx.scalarMultiply(s0);

            res.epoch = timeOfRover;
            res.nbrUnknowns = nbrUnknowns;
            res.nbrObs = nbrObservations;
            res.nbrIteration = nbr_iter;
            res.s0 = s0;
            res.sigma0 = sigma0;
            res.X_Rover = X_Rover;
            res.Kxx = Kxx;
            res.GDOP = GDOP;
            res.Kll = Kll;
            res.doubleDiffArrayList = doubleDiffArrayList;
            res.v = v;

            allResults.add(res);

        }


        private static class Results{
            public TimeE epoch;
            private int nbrObs ;

            private int nbrUnknowns ;

            private int nbrIteration ;
            private double sigma0;

            private double s0;

            private double GDOP;

            private RealMatrix Kll;

            private RealMatrix v;

            private RealMatrix X_Rover;

            private RealMatrix Kxx;

            private ArrayList<DoubleDiff> doubleDiffArrayList;

            private final Map<Integer,Iteration> iterations = new HashMap<>();

            public void mapIteration(Iteration iteration){

                iterations.put(iteration.id,iteration);
            }
        }
        static class Iteration{
            int id;
            RealMatrix A ;
        }

        private double computeObsEquation(RealMatrix X_Rover_init, DoubleDiff doubleDiff, Ephemeris.GpsNavMessageProto gpsNavMessageProto, TimeE timeOfRover, RealMatrix X_Base){
            int prn1 = doubleDiff.getPRN1();
            int prn2 = doubleDiff.getPRN2();

            RealMatrix X_prn1 = getSVPosition(prn1,gpsNavMessageProto,timeOfRover);
            RealMatrix X_prn2 = getSVPosition(prn2,gpsNavMessageProto,timeOfRover);

            // Approx range
            double r0_1 = X_prn1.subtract(X_Rover_init).getFrobeniusNorm();
            double r0_2 = X_prn2.subtract(X_Rover_init).getFrobeniusNorm();

            // t_emit
            double mjd_sec_of_day_emit_1 = timeOfRover.getMjd().doublePart - r0_1 / Constants.SPEED_OF_LIGHT;
            double mjd_sec_of_day_emit_2 = timeOfRover.getMjd().doublePart - r0_2 / Constants.SPEED_OF_LIGHT;

            TimeE.FormatTime mjd_emit_1 = new TimeE.FormatTime(timeOfRover.getMjd().integerPart, mjd_sec_of_day_emit_1);
            TimeE.FormatTime mjd_emit_2 = new TimeE.FormatTime(timeOfRover.getMjd().integerPart, mjd_sec_of_day_emit_2);

            TimeE timeOfEmit1 =  TimeE.fromMjd(mjd_emit_1);
            TimeE timeOfEmit2 =  TimeE.fromMjd(mjd_emit_2);

            // Satellite position at t_emit
            RealMatrix X_prn1_emit = getSVPosition(prn1, gpsNavMessageProto, timeOfEmit1);
            RealMatrix X_prn2_emit = getSVPosition(prn2, gpsNavMessageProto, timeOfEmit2);

            // Approx range
            r0_1 = X_prn1_emit.subtract(X_Rover_init).getFrobeniusNorm();
            r0_2 = X_prn2_emit.subtract(X_Rover_init).getFrobeniusNorm();

            // Earth rotation during propagation
            double alpha_1 = (r0_1 / Constants.SPEED_OF_LIGHT) * Constants.EARTH_ANGULAR_VELOCITY;
            double alpha_2 = (r0_2 / Constants.SPEED_OF_LIGHT) * Constants.EARTH_ANGULAR_VELOCITY;

            RealMatrix R_1 = new Array2DRowRealMatrix(new double[][] {
                    {Math.cos(alpha_1), Math.sin(alpha_1), 0},
                    {-Math.sin(alpha_1), Math.cos(alpha_1), 0},
                    {0, 0, 1.0}
            });

            RealMatrix R_2 = new Array2DRowRealMatrix(new double[][] {
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
            }


            double tTx = mes.getReceivedSvTimeNanos();
            double tRxGNSS = gnssClock.getTimeNanos() + mes.getTimeOffsetNanos() - (fullBiasNanos + BiasNanos);
            double tRx = tRxGNSS % Constants.NUMBER_NANO_SECONDS_WEEK;
            double pseudorange = (tRx - tTx) * 1e-9 * Constants.SPEED_OF_LIGHT;
            System.out.println("computePseudorange| state :" +mes.getState());
            System.out.println("computePseudorange| " + mes.getSvid() + " : " + pseudorange);

            return pseudorange;
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
        }

        private boolean hasEphemeris(Integer value){

            // Check Ephemeris is available for sv and refSv less than 2 hours ago
            System.currentTimeMillis();
            GpsTime systemGpsTime = GpsTime.now();
            Pair<Integer, Integer> systemGpsWeekSecond = systemGpsTime.getGpsWeekSecond();
            Integer systemSecond = systemGpsWeekSecond.second;

            Ephemeris.GpsNavMessageProto gpsNavMessageProto = mEphemerisManager.getmHardwareGpsNavMessageProto();

            if (gpsNavMessageProto!=null){

            for(Ephemeris.GpsEphemerisProto eph : gpsNavMessageProto.ephemerids){

                if ((eph.prn == value) &&
                        (eph.toe - systemSecond) < 7200){ // Time max since last ephemeris update : 2 hours
                    return true;
                }
            }}
            return false;
        }


        public RealMatrix getSVPosition(int prn, Ephemeris.GpsNavMessageProto mHardwareGpsNavMessageProto,TimeE timeE) {

            StringBuilder log = new StringBuilder(); // Create a StringBuilder for logging
            RealMatrix X_WGS84 = new BlockRealMatrix(3, 1);
            X_WGS84.setColumn(0, new double[]{0.0,0.0,0.0});

            // Check if PRN exists in ephemeris data
            Ephemeris.GpsEphemerisProto prnEphemeris;
            for (Ephemeris.GpsEphemerisProto eph : mHardwareGpsNavMessageProto.ephemerids) {
                if (eph.prn == prn) {
//                System.out.println("getSVPosition| Compute prn"+prn+" position");
                    // Get ephemeris data for the current SV
                    prnEphemeris = eph;

                    int gpsWeek = timeE.getGpsWeek();

                    // Delta temps
                    double dt = timeE.getTow() - prnEphemeris.toc;

//                if (prnEphemeris.week == gpsWeek) {
//                    String formattedDt = String.format("%.2f", dt / 3600);
//                    System.out.println("getSVPosition| at tGPS = " + timeE.getTow() + " (toe = " +prnEphemeris.toe + " )");
//                    System.out.println("getSVPosition| Time difference between epoch and ephemeris: " + formattedDt + " [hour]");
//                }

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

        public RealMatrix CHTRS95toMN95(RealMatrix X){

            //TODO : cart2ell, ell2EN
            RealMatrix t = new Array2DRowRealMatrix(new double[][] {
                    {-674.374},
                    {-15.056},
                    {-405.346}
            });

            return X.add(t);
        }

        public void saveDGPSResults() throws IOException {
            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss");
            Date currentdate = new Date();
            String filename = String.format("%s_%s.txt", "listing_CodeDifferential", formatter.format(currentdate));

            File textFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), filename);

            BufferedWriter bufferWriter;

            RealMatrix realX = new Array2DRowRealMatrix(new double[][] {
                    {4346401.6000},
                    {507454.7300},
                    {4625380.3000}
            });


            bufferWriter = new BufferedWriter(new FileWriter(textFile));

            StringBuilder stringBuilder = new StringBuilder();

            for(Results res : allResults){

                String str = String.format(Locale.US, "%20s %13.3f %13.3f %13.3f %9.3f %9.3f %9.3f",
                        res.epoch.getGpsTimeCalendar(),
                        res.X_Rover.getEntry(0,0),
                        res.X_Rover.getEntry(1,0),
                        res.X_Rover.getEntry(2,0),
                        Math.sqrt(Math.abs(res.Kxx.getEntry(0,0))),
                        Math.sqrt(Math.abs(res.Kxx.getEntry(1,0))),
                        Math.sqrt(Math.abs(res.Kxx.getEntry(2,0)))
                );
                String str2 = String.format(Locale.US, "Difference with real position %6.3f [m]",
                        res.X_Rover.subtract(realX).getFrobeniusNorm());
                stringBuilder.append(str).append("\n");
                stringBuilder.append(str2).append("\n");
            }

            bufferWriter.write(stringBuilder.toString());

            bufferWriter.close();
        }

        public void saveDetailedDGPSResults() throws IOException {
            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss");
            Date currentdate = new Date();
            String filename = String.format("%s_%s.txt", "detailed_listing_CodeDifferential", formatter.format(currentdate));

            File textFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), filename);

            BufferedWriter bufferWriter = new BufferedWriter(new FileWriter(textFile));

            StringBuilder stringBuilder = new StringBuilder();

            for(Results res : allResults){

                RealMatrix MN95 = CHTRS95toMN95(res.X_Rover);

                StringBuilder doubleDiffObsStringBuilder = new StringBuilder();

                for(DoubleDiff DD : res.doubleDiffArrayList){
                    double sigma_l = res.Kll.getEntry(DD.iObs-1, DD.iObs-1);
                    String str = String.format(Locale.US, "%3s %3s %3s %5s %3s %9.3f %9.3f %9.3f %13.3f %13.3f %13.3f %13.3f",
                            DD.getPRN1(), DD.getPRN2(), DD.getElevationPRN2(), 34, "C1C", //TODO: get signal auto
                            DD.getDoubleDiff(),
                            sigma_l,
                            res.v.getEntry(DD.iObs-1,0),
                            DD.zeroDiffBase.get(0), DD.zeroDiffBase.get(1),
                            DD.zeroDiffRover.get(0), DD.zeroDiffRover.get(1));
                    doubleDiffObsStringBuilder.append(str).append("\n");
                }


                String resBuilder = "---------------------------------------------------------------------------\n" +
                        "Differential Positioning (pseudo-distances) / Elisa Borlat / HEIG-VD\n" +
                        currentdate +" \n" +
                        "---------------------------------------------------------------------------" + "\n\n" +

                        "Global stats :" + "\n" +
                        "=======================" + "\n" +
                        "Nbr observations : " + res.nbrObs + "\n" +
                        "Nbr unknowns : " + res.nbrUnknowns + "\n" +
                        "Nbr iteration : " + res.nbrIteration + " (30)\n" +
                        "s0/sigma0 : " + res.s0 / res.sigma0 + "\n\n" +

                        "Epoch : " + "\n" +
                        "=======" + "\n" +
                        String.format("%-6s %-6s %6s %12s %2s %2s %4s %2s %2s %9s", "N0", "EPOCH", "MJD", "MJD     ", "DD", "MM", "YYYY", "hh", "mm", "ss.ssssss") + "\n" +
                        String.format("%-6s %-6s %6s %12s %10s %15s", "[-]", "[-]", "[day]", "[sec of day]", "[GPST]", "[GPST]") + "\n\n" +

                        "Double differences observations: " + "\n" +
                        "=================================" + "\n" +
                        String.format("%-4s %-4s %-4s %-5s %-3s %9s %9s %9s %13s %13s %13s %13s\n", "PRN1", "PRN2", "ePRN2", "EPOCH", "SIG", "DD OBS", "E.T", "v", "ZD(BASE,PRN1)", "ZD(BASE,PRN2)", "ZD(ROVER,PRN1)", "ZD(ROVER,PRN2)") +
                        String.format("%-4s %-4s %-4s %-5s %-3s %9s %9s %9s %13s %13s %13s %13s\n", "", "", "[°]", "", "", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]") +

                        doubleDiffObsStringBuilder + "\n" +

                        "Estimated parameters of the rover:" + "\n" +
                        "==================================" + "\n" +
                        String.format(Locale.US, "X WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(0,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(0,0)))) +
                        String.format(Locale.US, "Y WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(1,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(1,0)))) +
                        String.format(Locale.US, "Z WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(2,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(2,0)))) + "\n" + "\n" +

                        "Derivative parameters:" + "\n" +
                        "========================================" + "\n" +
                        String.format(Locale.US, "E MN95 : %13.3f [m] \n",
                                MN95.getEntry(0,0)) + // TODO : function MN95
                        String.format(Locale.US, "N MN95 : %13.3f [m] \n",
                                MN95.getEntry(1,0)) +
                        String.format(Locale.US, "h Bessel : %11.3f [m] \n\n",
                                MN95.getEntry(2,0)) +

                        String.format(Locale.US, "GDOP : %.1f [-]", res.GDOP);

                stringBuilder.append(resBuilder).append("\n");
            }

            bufferWriter.write(stringBuilder.toString());

            bufferWriter.close();
        }

        public void saveDetailedPlusDGPSResults() throws IOException {
            SimpleDateFormat formatter = new SimpleDateFormat("yyy_MM_dd_HH_mm_ss");
            Date currentdate = new Date();
            String filename = String.format("%s_%s.txt", "log", formatter.format(currentdate));

            File textFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), filename);

            BufferedWriter bufferWriter = new BufferedWriter(new FileWriter(textFile));

            StringBuilder stringBuilder = new StringBuilder();

            if (!allResults.isEmpty()){

            for(Results res : allResults){

                for(int i=1; i < res.nbrIteration; i++){
                    RealMatrix A = res.iterations.get(i).A;
                    stringBuilder.append(A);
                }


                RealMatrix MN95 = CHTRS95toMN95(res.X_Rover);

                StringBuilder doubleDiffObsStringBuilder = new StringBuilder();

                for(DoubleDiff DD : res.doubleDiffArrayList){
                    double sigma_l = res.Kll.getEntry(DD.iObs-1, DD.iObs-1);
                    String str = String.format(Locale.US, "%3s %3s %5s %3s %9.3f %9.3f %9.3f %13.3f %13.3f %13.3f %13.3f",
                            DD.getPRN1(), DD.getPRN2(), 34, "C1C", //TODO: get signal auto
                            DD.getDoubleDiff(),
                            sigma_l,
                            res.v.getEntry(DD.iObs-1,0),
                            DD.zeroDiffBase.get(0), DD.zeroDiffBase.get(1),
                            DD.zeroDiffRover.get(0), DD.zeroDiffRover.get(1));
                    doubleDiffObsStringBuilder.append(str).append("\n");
                }


                String resBuilder = "---------------------------------------------------------------------------\n" +
                        "Differential Positioning (pseudo-distances) / Elisa Borlat / HEIG-VD\n" +
                        currentdate +" \n" +
                        "---------------------------------------------------------------------------" + "\n\n" +

                        "Global stats :" + "\n" +
                        "=======================" + "\n" +
                        "Nbr observations : " + res.nbrObs + "\n" +
                        "Nbr unknowns : " + res.nbrUnknowns + "\n" +
                        "Nbr iteration : " + res.nbrIteration + " (30)\n" +
                        "s0/sigma0 : " + res.s0 / res.sigma0 + "\n\n" +

                        "Epoch : " + "\n" +
                        "=======" + "\n" +
                        String.format("%-6s %-6s %6s %12s %2s %2s %4s %2s %2s %9s", "N0", "EPOCH", "MJD", "MJD     ", "DD", "MM", "YYYY", "hh", "mm", "ss.ssssss") + "\n" +
                        String.format("%-6s %-6s %6s %12s %10s %15s", "[-]", "[-]", "[day]", "[sec of day]", "[GPST]", "[GPST]") + "\n\n" +

                        "Double differences observations: " + "\n" +
                        "=================================" + "\n" +
                        String.format("%-4s %-4s %-5s %-3s %9s %9s %9s %13s %13s %13s %13s\n", "PRN1", "PRN2", "EPOCH", "SIG", "DD OBS", "E.T", "v", "ZD(BASE,PRN1)", "ZD(BASE,PRN2)", "ZD(ROVER,PRN1)", "ZD(ROVER,PRN2)") +
                        String.format("%-4s %-4s %-5s %-3s %9s %9s %9s %13s %13s %13s %13s\n", "", "", "", "", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]", "[m]") +

                        doubleDiffObsStringBuilder + "\n" +

                        "Estimated parameters of the rover:" + "\n" +
                        "==================================" + "\n" +
                        String.format(Locale.US, "X WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(0,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(0,0)))) +
                        String.format(Locale.US, "Y WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(1,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(1,0)))) +
                        String.format(Locale.US, "Z WGS84 : %13.3f [m] +/- %4.3f [m] \n",
                                res.X_Rover.getEntry(2,0),
                                Math.sqrt(Math.abs(res.Kxx.getEntry(2,0)))) + "\n" + "\n" +

                        "Derivative parameters:" + "\n" +
                        "========================================" + "\n" +
                        String.format(Locale.US, "E MN95 : %13.3f [m] \n",
                                MN95.getEntry(0,0)) + // TODO : function MN95
                        String.format(Locale.US, "N MN95 : %13.3f [m] \n",
                                MN95.getEntry(1,0)) +
                        String.format(Locale.US, "h Bessel : %11.3f [m] \n\n",
                                MN95.getEntry(2,0)) +

                        String.format(Locale.US, "GDOP : %.1f [-]", res.GDOP);

                stringBuilder.append(resBuilder).append("\n");
            }}

            bufferWriter.write(stringBuilder.toString());

            bufferWriter.close();
        }





}
