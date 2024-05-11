package com.example.prototypedgps;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TableRow;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentStatusBinding;

public class StatusFragment extends Fragment {

    private FragmentStatusBinding binding;

    @Override
    public View onCreateView (@NonNull LayoutInflater inflater,
                              ViewGroup container,
                              Bundle savedInstanceState) {
        binding = FragmentStatusBinding.inflate(inflater, container, false);

        TableRow tableRow = new TableRow(getContext());

        TextView column1 = new TextView(getContext());
        TextView column2 = new TextView(getContext());
        column1.setText("6");
        column2.setText("34.7");
        tableRow.addView(column1);
        tableRow.addView(column2);

        binding.tableLayoutStatus.addView(tableRow);

        return binding.getRoot();
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

}
