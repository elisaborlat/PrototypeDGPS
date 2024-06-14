package com.example.prototypedgps;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;


import androidx.annotation.NonNull;

import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentMainBinding;

import java.io.IOException;
import java.util.Locale;
import java.util.stream.Stream;

public class HomeFragment extends Fragment {

    private FragmentMainBinding binding;

    private FileLogger mFileLogger;
    private RinexLogger mRinexLogger;

    private BaseStation mBaseStation;

    private RealTimePositionCalculator mRealTimePositionCalculator;
    private final HomeUIFragmentComponent mUiComponent = new HomeUIFragmentComponent();


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

            mUiComponent.setResultsCounter(0);


        });

        binding.buttonStopSend.setOnClickListener(v -> {
                    mFileLogger.send();
                    mRinexLogger.send();
                    if (binding.switchResults.isChecked()){
                        try {
                            mRealTimePositionCalculator.saveDGPSResults();
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                    if (binding.switchDetailedResults.isChecked()){
                        try {
                            mRealTimePositionCalculator.saveDetailedDGPSResults();
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
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

        return binding.getRoot();
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

        private int rinexCounter = 0;

        private int resultsCounter = 0;

        private final Handler handler = new Handler(Looper.getMainLooper());

        public void setRinexCounter(int rinexCount) {
            rinexCounter = rinexCount;
        }
        public void setResultsCounter(int resCounter) {
            resultsCounter = resCounter;
        }
        public void incrementRinexCounter() {
            handler.post(() -> {
                Activity activity = getActivity();
                rinexCounter ++;
                String resCounter = rinexCounter + " epochs";
                if (activity != null) {
                    binding.textView.setText(resCounter);
                }
            });
        }

        public void incrementResultCounter(){
            handler.post(() -> {
                Activity activity = getActivity();
                resultsCounter ++;
                String resCounter = resultsCounter + " epochs";
                if (activity != null) {
                    binding.textViewResCounter.setText(resCounter);
                }
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

        public void updateRes(int nbrSatObserved, int nbrObservationsGps, int nbrObservations, double pdop, double vdop, double hdop, double sigma) {
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textViewSat.setText(String.format(Locale.US, "%s/%s/%s", nbrObservations+1,nbrObservationsGps, nbrSatObserved));
                    binding.textViewPDOP.setText(String.format(Locale.US,"%.1f",pdop));
                    binding.textViewHVDOP.setText(String.format(Locale.US, "%.1f / %.1f", hdop,vdop));
                    binding.textViewSigma.setText(String.format(Locale.US, "%.2f", sigma));

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
    }

}
