package com.example.prototypedgps;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;


import androidx.annotation.NonNull;

import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentMainBinding;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Stream;

public class HomeFragment extends Fragment {

    private FragmentMainBinding binding;

    private FileLogger mFileLogger;
    private RinexLogger mRinexLogger;

    private BaseStation mBaseStation;

    private RealTimePositionCalculator mRealTimePositionCalculator;
    private final HomeUIFragmentComponent mUiComponent = new HomeUIFragmentComponent();

    public HomeFragment() {
    }


    @Override
    public View onCreateView (@NonNull LayoutInflater inflater,
                              ViewGroup container,
                              Bundle savedInstanceState) {

        if (mRinexLogger != null) {
            mRinexLogger.setUiFragmentComponent(mUiComponent);
        }
        if (mRealTimePositionCalculator != null) {
            mRealTimePositionCalculator.setUiFragmentComponent(mUiComponent);
        }
        if (mFileLogger != null) {
            mFileLogger.setUiFragmentComponent(mUiComponent);
        }


        binding = FragmentMainBinding.inflate(inflater, container, false);

        if(mBaseStation.isConnected()){
            binding.textViewConnected.setText(R.string.connected);
            binding.textViewConnected.setTextColor(Color.GREEN);
        }

        binding.buttonStartLog.setOnClickListener(v -> {

            binding.buttonStartLog.setEnabled(false);
            binding.buttonStopSend.setEnabled(true);
            Toast.makeText(getContext(), "Starting log...", Toast.LENGTH_LONG).show();

            binding.switchGnssMeasurement.setEnabled(false);
            binding.switchRinex.setEnabled(false);
            binding.switchResults.setEnabled(false);
            binding.switchDetailedResults.setEnabled(false);


            if (binding.switchRinex.isChecked()) {
                mRinexLogger.startNewLog();
            }

            if (binding.switchGnssMeasurement.isChecked()) {
                mFileLogger.startNewLog();
            }

            if (binding.switchDetailedResults.isChecked()) {
                mRealTimePositionCalculator.startNewLog();
            }

        });

        binding.buttonStopSend.setOnClickListener(v -> {
                    mFileLogger.send();
                    mRinexLogger.send();
                    mRealTimePositionCalculator.send();

            binding.buttonStopSend.setEnabled(false);
            binding.buttonStartLog.setEnabled(anySwitchOn());

                    binding.switchGnssMeasurement.setEnabled(true);
                    binding.switchRinex.setEnabled(true);
                    binding.switchResults.setEnabled(true);
                    binding.switchDetailedResults.setEnabled(true);
        }
        );

        binding.switchResults.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                binding.buttonStartLog.setEnabled(anySwitchOn());
            }
        });

        binding.switchGnssMeasurement.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            }
            else {
                binding.buttonStartLog.setEnabled(anySwitchOn());
            }
        });

        binding.switchDetailedResults.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                binding.buttonStartLog.setEnabled(anySwitchOn());
            }
        });

        binding.switchRinex.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                binding.buttonStartLog.setEnabled(anySwitchOn());
            }
        });

        binding.buttonpil6170.setOnClickListener(v -> {
            // isChecked = chtrs
            if (binding.toggleButton.isChecked()){
                String coordinateX = "4346325.847";
                String coordinateY = "507584.373";
                String coordinateZ = "4625432.121";
                binding.editTextNumberDecimalCoordx.setText(coordinateX);
                binding.editTextNumberDecimalCoordy.setText(coordinateY);
                binding.editTextNumberDecimalCoordz.setText(coordinateZ);
            }
            else{
                String coordinateX = "2540621.753";
                String coordinateY = "1181302.435";
                String coordinateZ = "447.764";
                binding.editTextNumberDecimalCoordx.setText(coordinateX);
                binding.editTextNumberDecimalCoordy.setText(coordinateY);
                binding.editTextNumberDecimalCoordz.setText(coordinateZ);
            }

        });

        binding.buttond01.setOnClickListener(v -> {
            // isChecked = chtrs
            if (binding.toggleButton.isChecked()){
                String coordinateX = "4346400.677";
                String coordinateY = "507454.190";
                String coordinateZ = "4625380.301";
                binding.editTextNumberDecimalCoordx.setText(coordinateX);
                binding.editTextNumberDecimalCoordy.setText(coordinateY);
                binding.editTextNumberDecimalCoordz.setText(coordinateZ);
            }
            else{
                String coordinateX = "2540483.004";
                String coordinateY = "1181225.169";
                String coordinateZ = "449.636";
                binding.editTextNumberDecimalCoordx.setText(coordinateX);
                binding.editTextNumberDecimalCoordy.setText(coordinateY);
                binding.editTextNumberDecimalCoordz.setText(coordinateZ);
            }
        });

        TextWatcher textWatcher = new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                boolean allFieldsFilled =  allCoordinateFieldsFilled();
                binding.switchRinex2.setEnabled(allFieldsFilled);
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        };

        binding.editTextNumberDecimalCoordx.addTextChangedListener(textWatcher);
        binding.editTextNumberDecimalCoordy.addTextChangedListener(textWatcher);
        binding.editTextNumberDecimalCoordz.addTextChangedListener(textWatcher);

        // Add a listener to handle toggle changes
        binding.toggleButton.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if(allCoordinateFieldsFilled()){
            double xValue = Double.parseDouble(binding.editTextNumberDecimalCoordx.getText().toString());
            double yValue = Double.parseDouble(binding.editTextNumberDecimalCoordy.getText().toString());
            double zValue = Double.parseDouble(binding.editTextNumberDecimalCoordz.getText().toString());

                // MN95 -> CHTRS
                if (isChecked) {
                    binding.textView15.setText("x");
                    binding.textView20.setText("y");
                    binding.textView21.setText("z");
                    double[] X = RealTimePositionCalculator.MN95toCHTRS(xValue,yValue,zValue);
                    String x = String.format(Locale.US,"%.3f",X[0]);
                    String y = String.format(Locale.US,"%.3f",X[1]);
                    String z = String.format(Locale.US,"%.3f",X[2]);
                    binding.editTextNumberDecimalCoordx.setText(x);
                    binding.editTextNumberDecimalCoordy.setText(y);
                    binding.editTextNumberDecimalCoordz.setText(z);
                } else {

                        binding.textView15.setText("E");
                        binding.textView20.setText("N");
                        binding.textView21.setText("h");
                        RealMatrix X = new Array2DRowRealMatrix(new double[][]{
                                {xValue},
                                {yValue},
                                {zValue}
                        });
                        double[] MN95 = RealTimePositionCalculator.CHTRS95toMN95(X);
                        String E = String.format(Locale.US,"%.3f",MN95[0]);
                        String N = String.format(Locale.US,"%.3f",MN95[1]);
                        String h = String.format(Locale.US,"%.3f",MN95[2]);
                        binding.editTextNumberDecimalCoordx.setText(E);
                        binding.editTextNumberDecimalCoordy.setText(N);
                        binding.editTextNumberDecimalCoordz.setText(h);
            }

        }});

        return binding.getRoot();
    }

    private boolean allCoordinateFieldsFilled(){
        return !binding.editTextNumberDecimalCoordx.getText().toString().trim().isEmpty() &&
                !binding.editTextNumberDecimalCoordy.getText().toString().trim().isEmpty() &&
                !binding.editTextNumberDecimalCoordz.getText().toString().trim().isEmpty();
    }

    private boolean anySwitchOn(){
        return Stream.of(binding.switchResults.isChecked(),
                binding.switchDetailedResults.isChecked(),
                binding.switchRinex.isChecked(),
                binding.switchGnssMeasurement.isChecked()).anyMatch(Boolean::booleanValue);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    public void setRealTimePositionCalculator(RealTimePositionCalculator mRealTimePositionCalculator) {
        this.mRealTimePositionCalculator = mRealTimePositionCalculator;
    }

    public void setFileLogger(FileLogger mFileLogger) {
        this.mFileLogger = mFileLogger;
    }

    public void setRinexLogger(RinexLogger mRinexLogger) {
        this.mRinexLogger = mRinexLogger;
    }

    public void setBaseStation(BaseStation mBaseStation) {
        this.mBaseStation = mBaseStation;
    }

    /** A facade for Home UI related operations. */
    public class HomeUIFragmentComponent{

        private final AtomicInteger rinexCounter = new AtomicInteger(0);
        private final AtomicInteger resultsCounter = new AtomicInteger(0);
        private final AtomicInteger rawCounter = new AtomicInteger(0);

        private final AtomicInteger detailedCounter = new AtomicInteger(0);


        private final Handler handler = new Handler(Looper.getMainLooper());

        public void resetRinexCounter() {
            rinexCounter.set(0);
        }

        public void resetRawCounter() {
            rawCounter.set(0);
        }

        public void resetDetailedCounter() {
            detailedCounter.set(0);
        }

        public void incrementRinexCounter() {
            updateCounter(rinexCounter, binding.textView);
        }


        public void incrementRawCounter(){
            updateCounter(rawCounter, binding.textViewRawCounter);
        }

        public void incrementDetailedResultsCounter(){
            updateCounter(detailedCounter, binding.textViewResCounterDetailedResults);
        }

        private void updateCounter(AtomicInteger counter, TextView textViewBinding) {
            handler.post(() -> {
                int currentCounter = counter.incrementAndGet();  // Increment the counter
                String resCounter = currentCounter+ " epochs";
                textViewBinding.setText(resCounter);
            });
        }

        public void updateEastNorthHBessel(double east,double north, double hBessel){
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textViewEast.setText(String.format(Locale.US, "%.3f m", east));
                    binding.textViewNorth.setText(String.format(Locale.US, "%.3f m", north));
                    binding.textViewH.setText(String.format(Locale.US, "%.3f m", hBessel));
                }
            });
        }

        public void updateRes(int nbrSatObserved, int nbrObservationsGps, int nbrObservations, double pdop, double vdop, double hdop, double sigma, double sigmaE, double sigmaN, double sigmaH) {
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textViewSat.setText(String.format(Locale.US, "%s/%s/%s", nbrObservations+1,nbrObservationsGps, nbrSatObserved));
                    binding.textViewPDOP.setText(String.format(Locale.US,"%.1f",pdop));
                    binding.textViewHVDOP.setText(String.format(Locale.US, "%.1f / %.1f", hdop,vdop));
                    binding.textViewSigma.setText(String.format(Locale.US, "%.2f", sigma));
                    binding.textViewSigmaE.setText(String.format(Locale.US, "%.2f", sigmaE));
                    binding.textViewSigmaN.setText(String.format(Locale.US, "%.2f", sigmaN));
                    binding.textViewSigmaH.setText(String.format(Locale.US, "%.2f", sigmaH));

                    if (sigma>3){
                        binding.textViewSigma.setTextColor(Color.RED);
                    } else {
                        binding.textViewSigma.setTextColor(Color.WHITE);
                    }

                }
            });
        }

        public void stationaryAntennaDecoded() {
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textViewDecoded.setText(String.format(Locale.US, "%s", "Position of base station decoded"));
                    binding.textViewDecoded.setTextColor(Color.GREEN);}
            });
        }

        public boolean isComputeConstraint(){
            Activity activity = getActivity();
            if (activity != null) {
            return binding.switchRinex2.isChecked();}
            return false;
        }

        public RealMatrix getCoordinateConstrainedPoint(){
            double coordinateX = Double.parseDouble(binding.editTextNumberDecimalCoordx.getText().toString());
            double coordinateY = Double.parseDouble(binding.editTextNumberDecimalCoordy.getText().toString());
            double coordinateZ = Double.parseDouble(binding.editTextNumberDecimalCoordz.getText().toString());
            //CHTRS
            if(binding.toggleButton.isChecked()){
                return new Array2DRowRealMatrix(new double[][]{
                        {coordinateX},
                        {coordinateY},
                        {coordinateZ}
            });
            } else {
                //Transform current MN95 coordinate to CHTRS
                double[] X_CHTRS = RealTimePositionCalculator.MN95toCHTRS(coordinateX,coordinateY,coordinateZ);
                return new Array2DRowRealMatrix(new double[][]{
                        {X_CHTRS[0]},
                        {X_CHTRS[1]},
                        {X_CHTRS[2]}
                });
            }
        }

        public void updateDeltaGroundTrue(RealMatrix X_Rover) {
            handler.post(() -> {
            //CHTRS
            if(binding.toggleButton.isChecked()){
                RealMatrix CHTRS = getCoordinateConstrainedPoint();
                RealMatrix deltaGroundTrue = CHTRS.subtract(X_Rover);
                binding.textViewDeltaGroundTrueX.setText(String.format(Locale.US, "%.3f m", deltaGroundTrue.getEntry(0,0)));
                binding.textViewDeltaGroundTrueY.setText(String.format(Locale.US, "%.3f m", deltaGroundTrue.getEntry(1,0)));
                binding.textViewDeltaGroundTrueZ.setText(String.format(Locale.US, "%.3f m", deltaGroundTrue.getEntry(2,0)));
            } else {
                double[] MN95GroundTrue = RealTimePositionCalculator.CHTRS95toMN95(getCoordinateConstrainedPoint());
                double[] MN95Compute = RealTimePositionCalculator.CHTRS95toMN95(X_Rover);
                double deltaGroundTrueEast = MN95GroundTrue[0] - MN95Compute[0];
                double deltaGroundTrueNorth = MN95GroundTrue[1] - MN95Compute[1];
                double deltaGroundTrueHeight = MN95GroundTrue[2] - MN95Compute[2];
                binding.textViewDeltaGroundTrueX.setText(String.format(Locale.US, "%.3f m", deltaGroundTrueEast));
                binding.textViewDeltaGroundTrueY.setText(String.format(Locale.US, "%.3f m", deltaGroundTrueNorth));
                binding.textViewDeltaGroundTrueZ.setText(String.format(Locale.US, "%.3f m", deltaGroundTrueHeight));
            }
            });


        }
    }

}
