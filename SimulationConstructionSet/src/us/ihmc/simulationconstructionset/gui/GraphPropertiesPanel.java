package us.ihmc.simulationconstructionset.gui;

import javafx.event.EventHandler;
import javafx.scene.control.*;
import javafx.scene.layout.GridPane;
import us.ihmc.simulationconstructionset.GraphConfiguration;

public class GraphPropertiesPanel extends GridPane implements EventHandler<javafx.event.ActionEvent> {
    private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

    private final TextField minTextField, maxTextField, baseLineTextField;
    private final RadioButton individualButton, autoButton, manualButton;

    private final RadioButton timePlotButton, phasePlotButton, baseLineButton;
    private final Button createFFTPlotButton;
    private final Button createBodePlotButton;

    private final YoGraph graph;
    private double newMinVal, newMaxVal;
    private double[] newBaseVal;

    public GraphPropertiesPanel(YoGraph graph) {
        super();

        this.graph = graph;

        // this.selectedVariable = variable;
        // selectedVariable.reCalcMinMax();
        newMinVal = graph.getGraphConfiguration().getManualScalingMin();
        newMaxVal = graph.getGraphConfiguration().getManualScalingMax();

        double[] baseLines = graph.getBaseLines();
        if ((baseLines != null) && (baseLines.length > 0)) {
            newBaseVal = baseLines;
        } else {
            newBaseVal = new double[]{0.0};
        }

        ToggleGroup scaleControlGroup = new ToggleGroup();
        ToggleGroup plotTypeGroup = new ToggleGroup();

        // Row 0:

        Label scalingLabel = new Label("Scaling:");
        GridPane.setConstraints(scalingLabel, 0, 0);

        individualButton = new RadioButton("Individual");
        individualButton.setSelected((graph.getGraphConfiguration().getScaleType() == GraphConfiguration.ScaleType.INDIVIDUAL));
        individualButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        individualButton.setToggleGroup(scaleControlGroup);
        GridPane.setConstraints(individualButton, 1, 0);

        autoButton = new RadioButton("Auto");
        autoButton.setSelected((graph.getGraphConfiguration().getScaleType() == GraphConfiguration.ScaleType.AUTO));
        autoButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        autoButton.setToggleGroup(scaleControlGroup);
        GridPane.setConstraints(autoButton, 3, 0);

        manualButton = new RadioButton("Manual");
        manualButton.setSelected((graph.getGraphConfiguration().getScaleType() == GraphConfiguration.ScaleType.MANUAL));
        manualButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        manualButton.setToggleGroup(scaleControlGroup);
        GridPane.setConstraints(manualButton, 5, 0);

        // Row 1:

        Label settingsLabel = new Label("Manual Settings:");
        GridPane.setConstraints(settingsLabel, 0, 1);

        Label minSettingsLabel = new Label("Min:");
        GridPane.setConstraints(minSettingsLabel, 1, 1);

        String minValString = numFormat.format(newMinVal);
        minTextField = new TextField(minValString);
        minTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        minTextField.setDisable(graph.getGraphConfiguration().getScaleType() != GraphConfiguration.ScaleType.MANUAL);
        GridPane.setConstraints(minTextField, 2, 1);

        Label maxSettingsLabel = new Label("Max:");
        GridPane.setConstraints(maxSettingsLabel, 4, 1);

        String maxValString = numFormat.format(newMaxVal);
        maxTextField = new TextField(maxValString);
        maxTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        maxTextField.setDisable(graph.getGraphConfiguration().getScaleType() != GraphConfiguration.ScaleType.MANUAL);
        GridPane.setConstraints(maxTextField, 5, 1);

        // Row 2:

        Label rangeLabel = new Label("Data Range:");
        GridPane.setConstraints(rangeLabel, 0, 2);

        Label minRangeLabel = new Label("Min:");
        GridPane.setConstraints(minRangeLabel, 1, 2);

        minValString = numFormat.format(graph.getMin());
        Label minTextLabel = new Label(minValString);
        GridPane.setConstraints(minTextLabel, 2, 2);

        Label maxRangeLabel = new Label("Max:");
        GridPane.setConstraints(maxRangeLabel, 4, 2);

        maxValString = numFormat.format(graph.getMax());
        Label maxTextLabel = new Label(maxValString);
        GridPane.setConstraints(maxTextLabel, 5, 2);

        // Row 3

        Label typeLabel = new Label("Plot Type:");
        GridPane.setConstraints(typeLabel, 0, 3);

        timePlotButton = new RadioButton("Time");
        timePlotButton.setSelected(graph.getGraphConfiguration().getGraphType() == GraphConfiguration.GraphType.TIME);
        timePlotButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        timePlotButton.setToggleGroup(plotTypeGroup);
        GridPane.setConstraints(timePlotButton, 1, 3);

        // TODO: can this be removed?
      /*
       * scatterPlotButton = new JRadioButton("Scatter",(graph.getGraphType()==graph.SCATTER_PLOT));
       * scatterPlotButton.addActionListener(this);
       * constraints.gridx = 3; constraints.gridy = 3;
       * constraints.gridwidth = 2;
       * gridbag.setConstraints(scatterPlotButton,constraints);
       * this.add(scatterPlotButton);
       */

        phasePlotButton = new RadioButton("Phase");
        phasePlotButton.setSelected(graph.getGraphConfiguration().getGraphType() == GraphConfiguration.GraphType.PHASE);
        phasePlotButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        phasePlotButton.setToggleGroup(plotTypeGroup);
        GridPane.setConstraints(phasePlotButton, 3, 3);

        createFFTPlotButton = new Button("FFT");
        createFFTPlotButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        GridPane.setConstraints(createFFTPlotButton, 4, 3);

        createBodePlotButton = new Button("Bode");
        createBodePlotButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        GridPane.setConstraints(createBodePlotButton, 5, 3);

        Label baseLineLabel = new Label("Base Line:");
        GridPane.setConstraints(baseLineLabel, 0, 4);

        StringBuilder baseLineValString = new StringBuilder();

        for (double current : newBaseVal) {
            baseLineValString.append(numFormat.format(current)).append(",");
        }

        baseLineTextField = new TextField(baseLineValString.toString());
        baseLineTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        baseLineTextField.setDisable(false);
        GridPane.setConstraints(baseLineTextField, 1, 4);


        baseLineButton = new RadioButton("Show Base Line");
        // TODO: this line was replaced in original code with setSelected(true) - is this line needed?
        //baseLineButton.setSelected(graph.getShowBaselines());
        baseLineButton.setSelected(true);
        baseLineButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        GridPane.setConstraints(baseLineButton, 3, 4);

        this.getChildren().addAll(
                scalingLabel,
                individualButton,
                autoButton,
                manualButton,
                settingsLabel,
                minSettingsLabel,
                minTextField,
                maxSettingsLabel,
                maxTextField,
                rangeLabel,
                minRangeLabel,
                minTextLabel,
                maxRangeLabel,
                maxTextLabel,
                typeLabel,
                timePlotButton,
                phasePlotButton,
                createFFTPlotButton,
                createBodePlotButton,
                baseLineLabel,
                baseLineTextField,
                baseLineButton
        );
    }

    public void commitChanges() {
        updateMinTextField();
        updateMaxTextField();
        updateBaseLineTextField();

        GraphConfiguration graphConfiguration = graph.getGraphConfiguration();

        graphConfiguration.setManualScalingMinMax(newMinVal, newMaxVal);

        if (this.individualButton.isSelected()) {
            graphConfiguration.setScaleType(GraphConfiguration.ScaleType.INDIVIDUAL);
        } else if (this.autoButton.isSelected()) {
            graphConfiguration.setScaleType(GraphConfiguration.ScaleType.AUTO);
        } else {
            graphConfiguration.setScaleType(GraphConfiguration.ScaleType.MANUAL);
        }

        if (this.timePlotButton.isSelected()) {
            graphConfiguration.setGraphType(GraphConfiguration.GraphType.TIME);
        } else if (this.phasePlotButton.isSelected()) {
            graphConfiguration.setGraphType(GraphConfiguration.GraphType.PHASE);
        }

        graphConfiguration.setShowBaselines(baseLineButton.isSelected());
        graphConfiguration.setBaselines(newBaseVal);
    }

    public void updateMaxTextField() {
        String text = maxTextField.getText();

        try {
            newMaxVal = Double.valueOf(text);
        } catch (NumberFormatException e) {
            maxTextField.setText(numFormat.format(newMaxVal));
        }

    }

    public void updateMinTextField() {
        String text = minTextField.getText();

        try {
            newMinVal = Double.valueOf(text);
        } catch (NumberFormatException e) {
            minTextField.setText(numFormat.format(newMinVal));
        }
    }

    public void updateBaseLineTextField() {
        String text = baseLineTextField.getText();

        try {
            newBaseVal = setBaseLines(text);

        } catch (NumberFormatException e) {
            StringBuilder baseLineValString = new StringBuilder();

            for (double current : newBaseVal) {
                baseLineValString.append(numFormat.format(current)).append(",");
            }

            baseLineTextField.setText(baseLineValString.toString());
        }
    }

    public double[] setBaseLines(String baseLinesCommaSeperated) {
        String[] lines = baseLinesCommaSeperated.split(",");
        double[] values = new double[lines.length];
        for (int i = 0; i < lines.length; i++) {
            // TODO: can the try block be removed?
//       try
//       {
            values[i] = Double.valueOf(lines[i]);

//       }
//       catch (NumberFormatException e)
//       {
//          values[i] = 0;
//       }
        }

        return values;
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == maxTextField) {
            updateMaxTextField();
        } else if (event.getSource() == minTextField) {
            updateMinTextField();
        } else if (event.getSource() == autoButton) {
            maxTextField.setDisable(true);
            minTextField.setDisable(true);
        } else if (event.getSource() == manualButton) {
            maxTextField.setDisable(false);
            minTextField.setDisable(false);
        } else if (event.getSource() == this.individualButton) {
            maxTextField.setDisable(true);
            minTextField.setDisable(true);
        } else if (event.getSource() == this.baseLineButton) {
            baseLineTextField.setDisable(!baseLineButton.isSelected());
        } else if (event.getSource() == this.createFFTPlotButton) {
            graph.createFFTPlotsFromEntriesBetweenInOutPoints();
        } else if (event.getSource() == this.createBodePlotButton) {
            graph.createBodePlotFromEntriesBetweenInOutPoints();
        }
    }
}
