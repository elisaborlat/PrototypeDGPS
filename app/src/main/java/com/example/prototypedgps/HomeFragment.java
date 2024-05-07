package com.example.prototypedgps;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;


import androidx.annotation.NonNull;

import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentMainBinding;

import java.util.stream.Stream;

public class HomeFragment extends Fragment {

    private FragmentMainBinding binding;

    private FileLogger mFileLogger;
    private RinexLogger mRinexLogger;

    public HomeFragment(FileLogger fileLogger, RinexLogger rinexLogger) {
        mFileLogger = fileLogger;
        mRinexLogger = rinexLogger;
    }


    @Override
    public View onCreateView (@NonNull LayoutInflater inflater,
                              ViewGroup container,
                              Bundle savedInstanceState) {
        binding = FragmentMainBinding.inflate(inflater, container, false);


        binding.buttonStartLog.setOnClickListener(v -> {
            if (binding.switchRinex.isChecked()) {
                mRinexLogger.startNewLog();
                Toast.makeText(getContext(), "Starting log...", Toast.LENGTH_LONG).show();
            }

            if (binding.switchGnssMeasurement.isChecked()) {
            mFileLogger.startNewLog();
                Toast.makeText(getContext(), "Starting log...", Toast.LENGTH_LONG).show();
            }

            binding.buttonStartLog.setEnabled(false);
            binding.buttonStopSend.setEnabled(true);
        });


        binding.buttonStopSend.setOnClickListener(v -> {
            if (binding.switchGnssMeasurement.isChecked()) {
                    mFileLogger.send();
            }
            if (binding.switchRinex.isChecked()) {
                mRinexLogger.send();
                    }
            binding.buttonStopSend.setEnabled(false);
            binding.buttonStartLog.setEnabled(anySwitchOn());
        }
        );

        binding.switch1.setOnCheckedChangeListener((buttonView, isChecked) -> {
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

        binding.switch2.setOnCheckedChangeListener((buttonView, isChecked) -> {
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
        return Stream.of(binding.switch1.isChecked(),
                binding.switch2.isChecked(),
                binding.switchRinex.isChecked(),
                binding.switchGnssMeasurement.isChecked()).anyMatch(Boolean::booleanValue);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

}
