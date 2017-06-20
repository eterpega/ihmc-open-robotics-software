package us.ihmc.simulationconstructionset.gui;

import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.ContextMenu;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.shape.Polyline;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.gui.dialogs.GraphPropertiesDialog;
import us.ihmc.yoVariables.dataBuffer.DataEntry;
import us.ihmc.yoVariables.dataBuffer.DataEntryHolder;
import us.ihmc.yoVariables.dataBuffer.TimeDataHolder;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;

public class YoGraph extends Pane implements EventHandler<Event> {
    private static final int DONT_PLOT_BOTTOM_PIXELS = 25;
    private static final int PIXELS_PER_BOTTOM_ROW = 14;    // 16;
    private static final int DONT_PLOT_TIMELINE_BOTTOM_PIXELS = 16;

    private GraphConfiguration graphConfiguration = new GraphConfiguration("default");

    protected static final int
            INDIVIDUAL_SCALING = GraphConfiguration.INDIVIDUAL_SCALING, AUTO_SCALING = GraphConfiguration.AUTO_SCALING,
            MANUAL_SCALING = GraphConfiguration.MANUAL_SCALING;
    protected static final int
            TIME_PLOT = GraphConfiguration.TIME_PLOT, PHASE_PLOT = GraphConfiguration.PHASE_PLOT;

    private final JFrame parentFrame;

    private final TimeDataHolder timeDataHolder;
    private final DataEntryHolder dataEntryHolder;
    private final GraphIndicesHolder graphIndicesHolder;
    private final YoGraphRemover yoGraphRemover;
    private final static int MAX_NUM_GRAPHS = 10;
    private final static int MAX_NUM_BASELINES = 6;
    private final static int VAR_NAME_SPACING_FOR_PRINT = 160;

    private final javafx.scene.paint.Color colors[] = new javafx.scene.paint.Color[YoGraph.MAX_NUM_GRAPHS];
    private final javafx.scene.paint.Color baseLineColors[] = new javafx.scene.paint.Color[YoGraph.MAX_NUM_BASELINES];

    private final ArrayList<DataEntry> entriesOnThisGraph;
    private final SelectedVariableHolder selectedVariableHolder;

    private double min = 0.0, max = 1.1;

    private double[] xData, yData;

    private final ArrayList<Integer> entryNamePaintWidths = new ArrayList<>();
    private final ArrayList<Integer> entryNamePaintRows = new ArrayList<>();
    private int totalEntryNamePaintRows = 1;
    private ContextMenu popupMenu;
    private javafx.scene.control.MenuItem delete;
    private static int actionPerformedByDragAndDrop = -1;
    private static Object sourceOfDrag = null;
    private static Object recipientOfDrag = null;
    boolean hadFocus = false;
    private boolean showNameSpace = false, showBaseLines = false;
    private int focusedBaseLine = 0;

    public YoGraph(GraphIndicesHolder graphIndicesHolder, YoGraphRemover yoGraphRemover, SelectedVariableHolder holder, DataEntryHolder dataEntryHolder, TimeDataHolder timeDataHolder, JFrame jFrame) {
        this.selectedVariableHolder = holder;
        this.dataEntryHolder = dataEntryHolder;
        this.timeDataHolder = timeDataHolder;

        xData = new double[0];
        yData = new double[0];

        this.graphIndicesHolder = graphIndicesHolder;
        this.yoGraphRemover = yoGraphRemover;
        this.parentFrame = jFrame;

        entriesOnThisGraph = new ArrayList<>();

        this.getChildren().addAll(
                new Canvas(),
                new Canvas(),
                new Canvas()
        );

        this.resetCanvasSizes();

        GridPane.setHgrow(this, Priority.ALWAYS);
        GridPane.setVgrow(this, Priority.ALWAYS);

        colors[0] = javafx.scene.paint.Color.rgb(0xa0, 0, 0);
        colors[1] = javafx.scene.paint.Color.rgb(0, 0, 0xff);
        colors[2] = javafx.scene.paint.Color.rgb(0, 0x80, 0);
        colors[3] = javafx.scene.paint.Color.rgb(0, 0, 0);

        colors[4] = javafx.scene.paint.Color.rgb(0x80, 0x80, 0x80);
        colors[5] = javafx.scene.paint.Color.rgb(0x80, 0, 0x80);
        colors[6] = javafx.scene.paint.Color.rgb(0, 0x80, 0x80);
        colors[7] = javafx.scene.paint.Color.rgb(0x60, 0x60, 0);
        colors[8] = javafx.scene.paint.Color.rgb(0xff, 0x50, 0x50);
        colors[9] = javafx.scene.paint.Color.rgb(0x50, 0xff, 0xff);

        baseLineColors[0] = javafx.scene.paint.Color.rgb(0x93, 0x70, 0xDB); // Purple
        baseLineColors[1] = javafx.scene.paint.Color.rgb(0x3C, 0xB3, 0x71); // Medium sea green
        baseLineColors[2] = javafx.scene.paint.Color.ORANGE;
        baseLineColors[3] = javafx.scene.paint.Color.ORANGE;
        baseLineColors[4] = javafx.scene.paint.Color.ORANGE;
        baseLineColors[5] = javafx.scene.paint.Color.ORANGE;

        this.addEventHandler(javafx.scene.input.MouseEvent.ANY, this);
        this.addEventHandler(javafx.scene.input.KeyEvent.ANY, this);

        //this.setDropTarget(new DropTarget(this, new YoGraphTargetListener(this))); TODO: replace

        popupMenu = new ContextMenu();

        this.setOnContextMenuRequested(e -> {
            popupMenu.show(this, this.getWidth(), this.getHeight());
        });

        delete = new javafx.scene.control.MenuItem("Delete Graph");
        delete.addEventHandler(Event.ANY, this);

        //this.setTransferHandler(new YoGraphTransferHandler()); TODO: replace
        this.showNameSpace = false;
    }

    public GraphConfiguration getGraphConfiguration() {
        return graphConfiguration;
    }

    protected int getScalingMethod() {
        return graphConfiguration.getScalingMethod();
    }

    protected void setScalingMethod(int method) {
        graphConfiguration.setScalingMethod(method);
    }

    protected int getPlotType() {
        return graphConfiguration.getPlotType();
    }

    protected void setPlotType(int type) {
        graphConfiguration.setPlotType(type);
    }

    protected double getManualMinScaling() {
        return graphConfiguration.getManualScalingMin();
    }

    protected double getManualMaxScaling() {
        return graphConfiguration.getManualScalingMax();
    }

    public void setShowBaseLines(boolean useBaseLine) {
        graphConfiguration.setShowBaseLines(useBaseLine);
    }

    public boolean getShowBaseLines() {
        return graphConfiguration.getShowBaseLines();
    }

    public void setBaseLines(double[] baseLines) {
        graphConfiguration.setBaseLines(baseLines);
    }

    public void incrementBaseLine(int baseLineIndex, double scale) {
        if (baseLineIndex >= graphConfiguration.getBaseLines().length)
            baseLineIndex = 0;

        double min = this.getMin();
        double max = this.getMax();

        double range = max - min;
        double amountToIncrement = 0.01 * range * scale;

        graphConfiguration.incrementBaseLine(baseLineIndex, amountToIncrement);
    }

    public void zeroBaseLine(int baseLineIndex) {
        if (baseLineIndex >= graphConfiguration.getBaseLines().length)
            baseLineIndex = 0;

        graphConfiguration.setBaseLine(baseLineIndex, 0.0);
    }

    public void centerBaseLine(int baseLineIndex) {
        if (baseLineIndex >= graphConfiguration.getBaseLines().length)
            baseLineIndex = 0;

        double min = this.getMin();
        double max = this.getMax();

        double center = (max + min) / 2.0;

        graphConfiguration.setBaseLine(baseLineIndex, center);
    }

    public double[] getBaseLines() {
        return graphConfiguration.getBaseLines();
    }

    protected double getMax() {
        reCalcMinMax();

        return this.max;
    }

    protected double getMin() {
        reCalcMinMax();

        return this.min;
    }

    protected void setGraphConfiguration(GraphConfiguration graphConfiguration) {
        if (graphConfiguration == null) {
            return;
        }

        setManualScaling(graphConfiguration.getManualScalingMin(), graphConfiguration.getManualScalingMax());
        setScalingMethod(graphConfiguration.getScalingMethod());
        setPlotType(graphConfiguration.getPlotType());

        setShowBaseLines(graphConfiguration.getShowBaseLines());
        setBaseLines(graphConfiguration.getBaseLines());
    }

    protected void setManualScaling(double minScaling, double maxScaling) {
        graphConfiguration.setManualScalingMinMax(minScaling, maxScaling);
    }

    public ArrayList<DataEntry> getEntriesOnThisGraph() {
        return entriesOnThisGraph;
    }

    public boolean isEmpty() {
        return entriesOnThisGraph.isEmpty();
    }

    public void setInteractionEnable(boolean enable) {
        if (enable) {
            // First remove all listeners in case they're already there:
            // Could also use this.getListeners(MouseListener); and remove those attached...

            this.removeEventHandler(javafx.scene.input.MouseEvent.ANY, this);
            this.removeEventHandler(javafx.scene.input.KeyEvent.ANY, this);

            // Then add them back.

            this.addEventHandler(javafx.scene.input.MouseEvent.ANY, this);
            this.addEventHandler(javafx.scene.input.KeyEvent.ANY, this);
        } else {
            // Just disable them and assume no repeats...

            this.removeEventHandler(javafx.scene.input.MouseEvent.ANY, this);
            this.removeEventHandler(javafx.scene.input.KeyEvent.ANY, this);
        }
    }

    public int getNumVars() {
        return this.entriesOnThisGraph.size();
    }

    private double previousGraphWidth;

    private void calculateRequiredEntryPaintWidthsAndRows() {
        entryNamePaintWidths.clear();
        entryNamePaintRows.clear();

        double graphWidth = this.getWidth();
        previousGraphWidth = graphWidth;

        int cumulatedWidth = 0;
        int row = 0;

        for (DataEntry entry : entriesOnThisGraph) {
            int variableWidth;

            if (showNameSpace) { variableWidth = (int) ((Canvas)this.getChildren().get(1)).getGraphicsContext2D().getFont().getSize()*entry.getFullVariableNameWithNameSpace().length(); }
            else { variableWidth = (int) ((Canvas)this.getChildren().get(1)).getGraphicsContext2D().getFont().getSize()*entry.getVariableName().length(); }

            int variablePlusValueWidth = variableWidth + 120;

            if ((cumulatedWidth != 0) && (cumulatedWidth + variablePlusValueWidth > graphWidth)) {
                row++;
                cumulatedWidth = 0;
            }

            cumulatedWidth += variablePlusValueWidth;

            entryNamePaintWidths.add(variablePlusValueWidth);
            entryNamePaintRows.add(row);
        }

        this.totalEntryNamePaintRows = row + 1;
    }

    public void addVariable(DataEntry entry) {
        if (entry == null) {
            return;
        }

        if (entriesOnThisGraph.size() >= YoGraph.MAX_NUM_GRAPHS)
            return;

        if (!entriesOnThisGraph.contains(entry))
            entriesOnThisGraph.add(entry);

        this.reCalcMinMax();
        calculateRequiredEntryPaintWidthsAndRows();

        this.repaintAllGraph();
    }

    public void addVariableFromSelectedVariableHolder() {
        YoVariable<?> yoVariable = selectedVariableHolder.getSelectedVariable();
        if (yoVariable != null)
            addVariable(dataEntryHolder.getEntry(yoVariable));
    }

    public void removeEntry(DataEntry entry) {
        if (entriesOnThisGraph.contains(entry))
            entriesOnThisGraph.remove(entry);

        this.reCalcMinMax();
        calculateRequiredEntryPaintWidthsAndRows();
    }

    boolean minMaxChanged() {
        boolean ret = false;

        int numVars = entriesOnThisGraph.size();
        if (numVars < 1) {
            return false;
        }

        for (DataEntry entry : entriesOnThisGraph) {
            ret = (ret || entry.minMaxChanged());
        }

        return ret;
    }

    private void reCalcMinMax() {
        int numVars = entriesOnThisGraph.size();

        if (numVars < 1) {
            return;
        }

        double newMin = Double.POSITIVE_INFINITY, newMax = Double.NEGATIVE_INFINITY;

        for (DataEntry entry : entriesOnThisGraph) {
            boolean inverted = entry.getInverted();

            double entryMin = entry.getMin();
            double entryMax = entry.getMax();

            if (inverted) {
                double temp = entryMax;
                entryMax = -entryMin;
                entryMin = -temp;
            }

            if (entryMax > newMax)
                newMax = entryMax;
            if (entryMin < newMin)
                newMin = entryMin;
        }

        this.min = newMin;
        this.max = newMax;
    }

    private int totalDontPlotBottomPixels = 0;

    private void calcXYData(DataEntry entry, int nPoints, double[] xData, double[] yData, int beginningAt) {
        double height = this.getHeight() - totalDontPlotBottomPixels;
        int leftPlotIndex = this.graphIndicesHolder.getLeftPlotIndex();

        double minVal = this.minFor(entry);

        double[] data = entry.getData(beginningAt, beginningAt + nPoints);

        for (int i = 0; i < nPoints; i++) {
            xData[i] = (((i + beginningAt) - leftPlotIndex) * this.getWidth()) / (this.graphIndicesHolder.getRightPlotIndex() - leftPlotIndex);
            yData[i] = height - (((entry.getInverted() ? -data[i] : data[i]) - minVal) / (this.maxFor(entry) - minVal) * height);
        }
    }

    private void calcScatterData(DataEntry entryX, DataEntry entryY, int nPoints, double[] xData, double[] yData, double minX, double maxX, double minY, double maxY, double width, double height, int offsetFromLeft, int offsetFromTop) {
        double[] dataX = entryX.getData();
        double[] dataY = entryY.getData();

        for (int i = 0; i < nPoints; i++) {
            xData[i] = ((dataX[i] - minX) / (maxX - minX) * width) + offsetFromLeft;
            yData[i] = height - (int) ((dataY[i] - minY) / (maxY - minY) * height) + offsetFromTop;
        }
    }

    private double[] zip(double[] a, double[] b) {
        if (a.length != b.length) {
            return new double[0];
        }

        double[] c = new double[a.length + b.length];

        for (int i = 0; i < a.length; ++i) {
            c[i * 2] = a[i];
            c[(i * 2) + 1] = b[i];
        }

        return c;
    }

    protected synchronized void printGraph(int printWidth, int printHeight) {
        int inPoint = graphIndicesHolder.getInPoint();
        int outPoint = graphIndicesHolder.getOutPoint();

        int numVars = entriesOnThisGraph.size();
        if (numVars == 0) {
            return;
        }

        int cumOffset = 3;

        // Here we use one scale for all the graphs:
        this.reCalcMinMax();

        for (int i = 0; i < numVars; i++) {
            cumOffset = i * ((int) (VAR_NAME_SPACING_FOR_PRINT * 0.6)) + 3;

            DataEntry entry = entriesOnThisGraph.get(i);
            double[] data = entry.getData();

            double minVal = 0.0, maxVal = 1.0;
            if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING) {
                if (entry.isAutoScaleEnabled()) {
                    minVal = entry.getMin();
                    maxVal = entry.getMax();
                } else {
                    minVal = entry.getManualMinScaling();
                    maxVal = entry.getManualMaxScaling();
                }
            } else if (graphConfiguration.getScalingMethod() == AUTO_SCALING) {
                minVal = this.min;
                maxVal = this.max;
            } else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING) {
                minVal = graphConfiguration.getManualScalingMin();
                maxVal = graphConfiguration.getManualScalingMax();
            }

            int nPoints = data.length;

            int length;
            length = (length = ((outPoint - inPoint + 1 + nPoints) % nPoints)) == 0 ? nPoints : length;

            double[] xDataPrint = new double[length];
            double[] yDataPrint = new double[length];

            for (int j = 0; j < length; j++) {
                int index = (inPoint + j) % nPoints;
                xDataPrint[j] = (j * printWidth) / length;
                yDataPrint[j] = (printHeight - DONT_PLOT_BOTTOM_PIXELS)
                        - (int) ((data[index] - minVal) / (maxVal - minVal) * (printHeight - DONT_PLOT_BOTTOM_PIXELS));
            }

            Polyline thisPoly = new Polyline(zip(xDataPrint, yDataPrint));
            thisPoly.setStroke(colors[i % YoGraph.MAX_NUM_GRAPHS]);
            this.getChildren().add(thisPoly);
        }
    }

    private void clearCanvases() {
        for (Node n : this.getChildren()) {
            if (n instanceof Canvas) {
                ((Canvas) n).getGraphicsContext2D().clearRect(0, 0, this.getWidth(), this.getHeight());
            }
        }
    }

    private void resetCanvasSizes() {
        for (Node n : this.getChildren()) {
            if (n instanceof Canvas) {
                ((Canvas) n).setHeight(this.getHeight());
                ((Canvas) n).setWidth(this.getWidth());
            }
        }
    }

    public void repaintAllGraph() {
        if (this.minMaxChanged()) {
            if (graphConfiguration.getScalingMethod() == AUTO_SCALING) {
                this.reCalcMinMax();
            }
        }

        this.resetCanvasSizes();

        this.clearCanvases();

        if (getPlotType() == TIME_PLOT) {
            paintTimePlot(graphIndicesHolder.getLeftPlotIndex(), graphIndicesHolder.getRightPlotIndex());
        } else if (getPlotType() == PHASE_PLOT) {
            paintPhasePlot();
        }
    }

    protected synchronized void repaintPartialGraph(int leftIndex, int rightIndex) {
        this.resetCanvasSizes();

        if (getPlotType() == TIME_PLOT) {
            paintTimePlot(leftIndex, rightIndex);
        } else if (getPlotType() == PHASE_PLOT) {
            paintPhasePlot();
        }
    }

    private StringBuffer stringBuffer = new StringBuffer(80);
    @SuppressWarnings("unused")
    private String spaceString = "  ";
    private char[] charArray = new char[80];
    private final java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");
    private final FieldPosition fieldPosition = new FieldPosition(NumberFormat.INTEGER_FIELD);

    public void createBodePlotFromEntriesBetweenInOutPoints() {
        if (entriesOnThisGraph.size() < 2) {
            System.out.println("need 2 entries (input/output) for Bode plot");
            return;
        }
        if (!checkInOutPoints())
            return;

        int inPoint = graphIndicesHolder.getInPoint();
        int outPoint = graphIndicesHolder.getOutPoint();

        DataEntry input = entriesOnThisGraph.get(0);
        DataEntry output = entriesOnThisGraph.get(1);

        double[] inputData = Arrays.copyOfRange(input.getData(), inPoint, outPoint);
        double[] outputData = Arrays.copyOfRange(output.getData(), inPoint, outPoint);

        double[] timeData = Arrays.copyOfRange(timeDataHolder.getTimeData(), inPoint, outPoint);

        BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
    }

    public void createBodePlotFromEntries() {
        if (entriesOnThisGraph.size() < 2)
            return;

        DataEntry input = entriesOnThisGraph.get(0);
        DataEntry output = entriesOnThisGraph.get(1);

        double[] inputData = input.getData();
        double[] outputData = output.getData();

        double[] timeData = timeDataHolder.getTimeData();

        BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
    }

    private boolean checkInOutPoints() {
        int inPoint = graphIndicesHolder.getInPoint();
        int outPoint = graphIndicesHolder.getOutPoint();

        boolean valid = outPoint > inPoint;

        if (!valid) {
            System.out.println("Please set inPoint < outPoint and re-try");
        }

        return valid;
    }

    public void createFFTPlotsFromEntriesBetweenInOutPoints() {
        if (!checkInOutPoints()) return;

        int inPoint = graphIndicesHolder.getInPoint();
        int outPoint = graphIndicesHolder.getOutPoint();

        double[] timeData = timeDataHolder.getTimeData();
        double[] rngTimeData = Arrays.copyOfRange(timeData, inPoint, outPoint);

        for (DataEntry entry : entriesOnThisGraph) {
            double[] data = entry.getData();
            double[] rngData = Arrays.copyOfRange(data, inPoint, outPoint);

            BodePlotConstructor.plotFFT(entry.getVariableName(), rngTimeData, rngData);
        }
    }


    public void createFFTPlotsFromEntries() {
        double[] timeData = timeDataHolder.getTimeData();

        for (DataEntry entry : entriesOnThisGraph) {
            double[] data = entry.getData();

            BodePlotConstructor.plotFFT(entry.getVariableName(), timeData, data);
        }
    }

    double minFor(DataEntry entry) {
        double minVal = 0.0;
        if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING) {
            if (entry.isAutoScaleEnabled()) {
                minVal = entry.getMin();
            } else {
                minVal = entry.getManualMinScaling();
            }
        } else if (graphConfiguration.getScalingMethod() == AUTO_SCALING) {
            this.reCalcMinMax();
            minVal = this.min;
        } else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING) {
            minVal = graphConfiguration.getManualScalingMin();
        }
        return minVal;
    }

    double maxFor(DataEntry entry) {
        double maxVal = 0.0;
        if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING) {
            if (entry.isAutoScaleEnabled()) {
                maxVal = entry.getMax();
            } else {
                maxVal = entry.getManualMaxScaling();
            }
        } else if (graphConfiguration.getScalingMethod() == AUTO_SCALING) {
            this.minMaxChanged();
            maxVal = this.max;
        } else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING) {
            maxVal = graphConfiguration.getManualScalingMax();
        }
        return maxVal;
    }

    public void paintPhasePlot() {
        double graphWidth = this.getWidth();
        double graphHeight = this.getHeight();

        int numVars = entriesOnThisGraph.size();

        GraphicsContext gc = ((Canvas) this.getChildren().get(0)).getGraphicsContext2D();

        for (int i = 0; i < entriesOnThisGraph.size() / 2; i++) {
            DataEntry entryX = entriesOnThisGraph.get(i);
            double[] dataX = entryX.getData();

            DataEntry entryY = entriesOnThisGraph.get(i + 1);

            double minValX = 0.0, maxValX = 1.0;
            double minValY = 0.0, maxValY = 1.0;

            if (graphConfiguration.getScalingMethod() == INDIVIDUAL_SCALING) {
                if (entryX.isAutoScaleEnabled()) {
                    minValX = entryX.getMin();
                    maxValX = entryX.getMax();
                } else {
                    minValX = entryX.getManualMinScaling();
                    maxValX = entryX.getManualMaxScaling();
                }

                if (entryY.isAutoScaleEnabled()) {
                    minValY = entryY.getMin();
                    maxValY = entryY.getMax();
                } else {
                    minValY = entryY.getManualMinScaling();
                    maxValY = entryY.getManualMaxScaling();
                }
            } else if (graphConfiguration.getScalingMethod() == AUTO_SCALING) {
                // minValX = minValY = this.min;   //++++++
                // maxValX = maxValY = this.max;

                minValY = entryY.getMin();
                maxValY = entryY.getMax();

                minValX = entryX.getMin();
                maxValX = entryX.getMax();
            } else if (graphConfiguration.getScalingMethod() == MANUAL_SCALING) {
                minValX = minValY = graphConfiguration.getManualScalingMin();    // ++++++
                maxValX = maxValY = graphConfiguration.getManualScalingMax();
            }

            int nPoints = dataX.length;
            if ((xData.length != nPoints) || (yData.length != nPoints)) {
                xData = new double[nPoints];
                yData = new double[nPoints];
            }

            int totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);

            calcScatterData(entryX, entryY, nPoints, xData, yData, minValX, maxValX, minValY, maxValY, (graphWidth - 6), graphHeight - totalDontPlotBottomPixels,
                    3, 5);

            gc.setStroke(colors[i % YoGraph.MAX_NUM_GRAPHS]);
            gc.strokePolyline(xData, yData, xData.length);

            // Draw a Cross Hairs:
            int index = graphIndicesHolder.getIndex();

            if ((index < xData.length) && (index < yData.length) & (index >= 0)) {
                gc.setStroke(javafx.scene.paint.Color.BLACK);
                gc.strokeLine(xData[index] - 5, yData[index], xData[index] + 5, yData[index]);
                gc.strokeLine(xData[index], yData[index] - 10, xData[index], yData[index] + 10);
            }
        }

        paintVariableNamesAndValues(true);
    }

    public void paintTimePlot(int leftPlotIndex, int rightPlotIndex) {
        double graphWidth = this.getWidth();
        double graphHeight = this.getHeight();

        if (this.minMaxChanged() && graphConfiguration.getScalingMethod() == AUTO_SCALING) {
            leftPlotIndex = graphIndicesHolder.getLeftPlotIndex();
        }

        if (graphWidth != previousGraphWidth) {
            calculateRequiredEntryPaintWidthsAndRows();
        }

        int numVars = entriesOnThisGraph.size();

        GraphicsContext gc = ((Canvas) this.getChildren().get(0)).getGraphicsContext2D();

        for (int i = 0; i < numVars; i++) {
            DataEntry entry = entriesOnThisGraph.get(i);

            int entryLeftIndex = leftPlotIndex;
            int entryRightIndex = rightPlotIndex;

            if (entry.havePointsChanged()) {
                entryLeftIndex = graphIndicesHolder.getLeftPlotIndex();
                entryRightIndex = graphIndicesHolder.getRightPlotIndex();
            }

            if (entry.havePointsChanged() || entry.hasDataChanged() || entry.minMaxChanged()) {
                int nPoints = entryRightIndex - entryLeftIndex;
                if ((xData.length != nPoints) || (yData.length != nPoints)) {
                    xData = new double[nPoints];
                    yData = new double[nPoints];
                }

                totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);

                calcXYData(entry, nPoints, xData, yData, entryLeftIndex);

                if (xData.length > 1) {
                    final int x = i;
                    final double[] xD = xData.clone();
                    final double[] yD = yData.clone();
                    Platform.runLater(() -> {
                        if (x == 0) gc.clearRect(Math.ceil(xD[0]), 0, Math.floor(xD[xD.length - 1] - xD[0]), this.getHeight());
                        gc.setStroke(colors[x % YoGraph.MAX_NUM_GRAPHS]);
                        for (int z = 0; z < xD.length-1; ++z) {
                            gc.strokeLine(xD[z], yD[z], xD[z+1], yD[z+1]);
                        }
                    });
                }

                if (graphConfiguration.getShowBaseLines()) {
                    double[] baseLines = graphConfiguration.getBaseLines();

                    for (int j = 0; j < baseLines.length; j++) {
                        double baseLine = baseLines[j];
                        final int baseY = (int) (graphHeight - totalDontPlotBottomPixels) - (int) ((baseLine - this.minFor(entry)) / (this.maxFor(entry) - this.minFor(entry)) * (graphHeight - totalDontPlotBottomPixels)) + 5;

                        final int x = j;
                        Platform.runLater(() -> {
                            GraphicsContext gc2 = ((Canvas) this.getChildren().get(1)).getGraphicsContext2D();
                            gc2.clearRect(0, 0, this.getWidth(), this.getHeight());
                            gc2.setStroke(baseLineColors[x]);
                            gc2.strokeLine(0, baseY, this.getWidth(), baseY);
                        });
                    }
                }

                entry.resetDataChanged();
                entry.resetMinMaxChanged();
                entry.resetPointsChanged();
            }
        }

        Platform.runLater(() -> {
            paintVerticalIndexLines();
            paintVariableNamesAndValues(false);
        });
    }

    void paintVerticalIndexLines() {
        double width = this.getWidth();
        double height = this.getHeight() - totalDontPlotBottomPixels;
        int inPoint = this.graphIndicesHolder.getInPoint();
        int outPoint = this.graphIndicesHolder.getOutPoint();
        int leftIndex = this.graphIndicesHolder.getLeftPlotIndex();
        int rightIndex = this.graphIndicesHolder.getRightPlotIndex();

        GraphicsContext gc = ((Canvas) this.getChildren().get(2)).getGraphicsContext2D();
        gc.clearRect(0, 0, width, height);
        gc.setLineWidth(1.5f);

        double linex;

        if (inPoint >= leftIndex) {
            linex = (((inPoint - leftIndex) * width) / (rightIndex - leftIndex));
            gc.setStroke(javafx.scene.paint.Color.GREEN);
            gc.strokeLine(linex, 0, linex, height);
        }

        if (outPoint <= rightIndex) {
            linex = (((outPoint - leftIndex) * width) / (rightIndex - leftIndex));
            gc.setStroke(javafx.scene.paint.Color.RED);
            gc.strokeLine(linex, 0, linex, height);
        }

        {
            ArrayList<Integer> keys = graphIndicesHolder.getKeyPoints();

            for (int i = 0; i < keys.size(); i++) {
                int value = keys.get(i);
                if (value >= leftIndex && value <= rightIndex) {
                    linex = ((value * width) / (rightIndex - leftIndex));
                    gc.setStroke(javafx.scene.paint.Color.ORANGE);
                    gc.strokeLine(linex, 0, linex, height);
                }
            }
        }

        {
            linex = ((this.graphIndicesHolder.getIndex() * width) / (rightIndex - leftIndex));
            gc.setStroke(javafx.scene.paint.Color.BLACK);
            gc.strokeLine(linex, 0, linex, height);
        }
    }

    void paintVariableNamesAndValues(boolean phasePlot) {
        int previousRow = 0;
        int cumulativeOffset = 3;

        GraphicsContext gc = ((Canvas) this.getChildren().get(2)).getGraphicsContext2D();

        gc.clearRect(0, this.getHeight() - totalDontPlotBottomPixels, this.getWidth(), totalDontPlotBottomPixels);

        if (showBaseLines) {
            drawBaseLines();
            return;
        }

        for (int i = 0; i < entriesOnThisGraph.size(); i++) {
            DataEntry entry = entriesOnThisGraph.get(i);

            if (phasePlot) {
                gc.setFill(colors[i / 2 % YoGraph.MAX_NUM_GRAPHS]);
            } else {
                gc.setFill(colors[i % YoGraph.MAX_NUM_GRAPHS]);
            }

            // Draw the variable name
            int row = entryNamePaintRows.get(i);
            if (row != previousRow) {
                cumulativeOffset = 3;
                previousRow = row;
            }

            drawVariableNameAndValue(cumulativeOffset, row, entry);
            cumulativeOffset += entryNamePaintWidths.get(i);
        }
    }

    private void drawBaseLines() {
        if (!graphConfiguration.getShowBaseLines()) return;

        double[] baseLines = graphConfiguration.getBaseLines();
        if (baseLines.length == 0) return;

        int row = 0;
        double graphHeight = this.getHeight();
        int yToDrawAt = (int) graphHeight - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - row - 1));

        int cumOffset = 3;

        double total = 0.0;

        GraphicsContext gc = ((Canvas) this.getChildren().get(1)).getGraphicsContext2D();
        gc.clearRect(0,this.getHeight()-yToDrawAt, this.getWidth(), yToDrawAt);

        gc.setFill(javafx.scene.paint.Color.BLACK);
        gc.fillText("BaseLines: ", cumOffset, yToDrawAt);
        cumOffset += 80;

        gc.setLineWidth(1.5f);
        for (int i = 0; i < baseLines.length; i++) {
            stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...
            double baseLine = baseLines[i];
            total = total + baseLine;

            formatDouble(stringBuffer, baseLine);
            String baseLineString = stringBuffer.toString();

            int baseLineStringWidth = (int) gc.getFont().getSize() * baseLineString.length();

            gc.setFill(colors[i]);
            gc.fillText(baseLineString, cumOffset, yToDrawAt);
            cumOffset = cumOffset + baseLineStringWidth + 10;
        }

        double average = total / ((double) baseLines.length);

        stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...
        formatDouble(stringBuffer, average);
        String averageString = stringBuffer.toString();

        gc.setFill(javafx.scene.paint.Color.BLACK);
        gc.fillText("    Average = " + averageString, cumOffset, yToDrawAt);
    }

    private void drawVariableNameAndValue(int cumOffset, int row, DataEntry entry) {
        double graphHeight = this.getHeight();

        GraphicsContext gc = ((Canvas) this.getChildren().get(2)).getGraphicsContext2D();

        stringBuffer.delete(0, stringBuffer.length());    // Erase the string buffer...

        // +++JEP 12/3/2012: Draw the value at the index, not the current YoVariable value. That way if a robot thread is updating
        // Sensor information, you see the saved value, not the value as it's being updated.

        if (graphIndicesHolder.isIndexAtOutPoint()) {
            entry.getVariableNameAndValue(stringBuffer);
        } else {
            entry.getVariableNameAndValueAtIndex(stringBuffer, graphIndicesHolder.getIndex());
        }

        if (showNameSpace) {
            NameSpace nameSpace = entry.getVariable().getNameSpace();
            stringBuffer.insert(0, nameSpace);
        }

        int length = Math.min(stringBuffer.length(), charArray.length);
        stringBuffer.getChars(0, length, charArray, 0);    // dump it into the character Array

        int yToDrawAt = (int) graphHeight - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - row - 1));

        gc.fillText(stringBuffer.toString(), cumOffset, yToDrawAt);    // Print it.
    }

    public void handleKeyPressed(javafx.scene.input.KeyEvent evt) {
        KeyCode code = evt.getCode();

        switch (code) {
            case LEFT:
                this.graphIndicesHolder.tickLater(-1);
                break;
            case RIGHT:
                this.graphIndicesHolder.tickLater(1);
                break;
            case ALT:
                showNameSpace = true;
                calculateRequiredEntryPaintWidthsAndRows();
                break;
            case CONTROL:
                showBaseLines = true;
                calculateRequiredEntryPaintWidthsAndRows();
                break;
            case UP:
                incrementBaseLine(focusedBaseLine, 1.0);
                break;
            case DOWN:
                incrementBaseLine(focusedBaseLine, -1.0);
                break;
        }
    }

    public void handleKeyReleased(javafx.scene.input.KeyEvent evt) {
        KeyCode code = evt.getCode();

        switch (code) {
            case ALT: {
                showNameSpace = false;
                calculateRequiredEntryPaintWidthsAndRows();
                break;
            }
            case CONTROL: {
                showBaseLines = false;
                calculateRequiredEntryPaintWidthsAndRows();
                break;
            }
        }
    }

    public void handleKeyTyped(javafx.scene.input.KeyEvent evt) {
        String character = evt.getCharacter();

        switch (character) {
            case "1":
                this.focusedBaseLine = 0;
                break;
            case "2":
                this.focusedBaseLine = 1;
                break;
            case "3":
                this.focusedBaseLine = 2;
                break;
            case "4":
                this.focusedBaseLine = 3;
                break;
            case "5":
                this.focusedBaseLine = 4;
                break;
            case "6":
                this.focusedBaseLine = 5;
                break;
            case "z":
                zeroBaseLine(focusedBaseLine);
                break;
            case "c":
                centerBaseLine(focusedBaseLine);
                break;
        }
    }

    private void handleMousePressed(javafx.scene.input.MouseEvent evt) {
        this.requestFocus();
        double y = evt.getY();
        double x = evt.getX();
        double h = getHeight();
        double w = getWidth();

        // Remember stuff for drag...
        this.clickedX = x;
        this.clickedY = y;
        this.draggedX = x;
        this.draggedY = y;
        this.clickedIndex = clickIndex(x, w);
        this.clickedLeftIndex = graphIndicesHolder.getLeftPlotIndex();
        this.clickedRightIndex = graphIndicesHolder.getRightPlotIndex();

        // Double click brings up var properties dialog box.
        if ((evt.getClickCount() == 2) && (!entriesOnThisGraph.isEmpty())) {
            if (parentFrame != null) {
                GraphPropertiesDialog dialog = new GraphPropertiesDialog(parentFrame, this);
                dialog.show();

                // parentFrame.repaint(); // This is a horrible way to get the graphs to repaint...
            }

        }

        // Right click places and deletes graphs:

        // if (evt.isControlDown())
        // if (evt.isMetaDown() && evt.isAltDown())
        // if (evt.isMetaDown() && evt.isControlDown())
        // if (evt.isShiftDown())
        if (evt.isMiddleButtonDown()) {    // Middle Click
            // If mouse was pressed in a label, remove that variable:

            if (y > h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW) {
                int idx = getClickedVariableIndex(x, y, h);

                if (idx < this.entriesOnThisGraph.size()) {
                    this.removeEntry(this.entriesOnThisGraph.get(idx));
                }
            } else {
                addVariableFromSelectedVariableHolder();
            }
        }

        // Left click places index:

        else if (evt.isPrimaryButtonDown()) {
            if ((this.entriesOnThisGraph == null) || (this.entriesOnThisGraph.size() < 1) || (getPlotType() == PHASE_PLOT)) {
                return;
            }

            if (y <= h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW) {
                int newIndex = clickIndex(x, w);

                // if (nPoints > 0)
                this.graphIndicesHolder.setIndexLater(newIndex);

                // this.graphArrayPanel.setIndexLater((x*nPoints)/w);
            }

            // If mouse was pressed in a label, highlight that variable and initiate drag and drop:
            else {
                int index = getClickedVariableIndex(x, y, h);

                if (index < this.entriesOnThisGraph.size()) {
                    DataEntry entry = entriesOnThisGraph.get(index);
                    selectedVariableHolder.setSelectedVariable(entry.getVariable());

                    if (!evt.isControlDown()) {
                        //this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.MOVE);
                        actionPerformedByDragAndDrop = TransferHandler.MOVE;
                    } else if (evt.isControlDown()) {
                        //this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.COPY);
                        actionPerformedByDragAndDrop = TransferHandler.COPY;
                    }

                    sourceOfDrag = this;
                }
            }

        } else if (evt.isSecondaryButtonDown()) {
            //popupMenu.setLocation((int)evt.getScreenX(), (int)evt.getScreenY());

            popupMenu.getItems().clear();

            for (final DataEntry dataBufferEntry : entriesOnThisGraph) {
                final javafx.scene.control.MenuItem menuItem = new javafx.scene.control.MenuItem("Remove " + dataBufferEntry.getVariableName());

                menuItem.addEventHandler(ActionEvent.ANY, e -> {
                    removeEntry(dataBufferEntry);
                    repaintAllGraph();
                    popupMenu.getItems().remove(menuItem);
                    popupMenu.hide();
                });

                popupMenu.getItems().add(menuItem);
            }

            popupMenu.getItems().add(delete);
        }
    }

    private int getClickedVariableIndex(double x, double y, double graphHeight) {
        int rowClicked = (int) (y - (graphHeight - totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)) / PIXELS_PER_BOTTOM_ROW;

        // System.out.println("rowClicked = " + rowClicked);

        int index = 0;
        int totalOffset = 3;

        for (int j = 0; j < entryNamePaintWidths.size(); j++) {
            Integer entryPaintWidth = entryNamePaintWidths.get(j);
            Integer row = entryNamePaintRows.get(j);

            if (row < rowClicked) {
                index++;
            } else {
                totalOffset = totalOffset + entryPaintWidth;
                if (x > totalOffset)
                    index++;
            }
        }

        return index;
    }

    private int clickIndex(double x, double w) {
        int leftPlotIndex = graphIndicesHolder.getLeftPlotIndex();
        int rightPlotIndex = graphIndicesHolder.getRightPlotIndex();

        return clickIndex(x, w, leftPlotIndex, rightPlotIndex);    // (leftPlotIndex + (2*x*(rightPlotIndex-leftPlotIndex)+w)/(2*w));
    }

    private int clickIndex(double x, double w, int leftPlotIndex, int rightPlotIndex) {
        return (int) (leftPlotIndex + (2 * x * (rightPlotIndex - leftPlotIndex) + w) / (2 * w));
    }

    @SuppressWarnings("unused")
    private double clickedX, clickedY;
    @SuppressWarnings("unused")
    private double draggedX, draggedY;
    private int clickedIndex, clickedLeftIndex, clickedRightIndex;

    public void handleMouseDragged(javafx.scene.input.MouseEvent evt) {
        draggedX = evt.getX();
        draggedY = evt.getY();

        double h = getHeight();
        double w = getWidth();

        if (draggedX > w)
            draggedX = w;
        if (draggedX < 0)
            draggedX = 0;

        if (clickedY > h - DONT_PLOT_TIMELINE_BOTTOM_PIXELS)
            return;

        if (!evt.isMetaDown() && (!evt.isAltDown()) && (getPlotType() != PHASE_PLOT)) {    // Left Click n Drag
            int index = clickIndex(draggedX, w, clickedLeftIndex, clickedRightIndex);
            graphIndicesHolder.setIndexLater(index);    // +++JEP setIndex or setIndexLater??

            // graphArrayPanel.setIndex(index); //+++JEP setIndex or setIndexLater??
            // graphArrayPanel.repaintGraphs();
            // clickIndex = index;

            // System.out.println("x: " + draggedX + "index: " + index);
        }

        if (evt.isMetaDown() && (!evt.isAltDown())) {    // Right Click n Drag
            // draggedX = evt.getX();

            // if (draggedX > w) draggedX = w;
            // if (draggedX < 0) draggedX = 0;

            // System.out.println(draggedX);
            int index = clickIndex(draggedX, w, clickedLeftIndex, clickedRightIndex);

            // System.out.println("Mouse Dragged!!");

            int newLeftIndex = clickedLeftIndex + clickedIndex - index;
            int newRightIndex = clickedRightIndex + clickedIndex - index;

            if (newLeftIndex < 0) {
                newLeftIndex = 0;
                newRightIndex = clickedRightIndex - clickedLeftIndex;
            }

            if (newRightIndex > graphIndicesHolder.getMaxIndex()) {
                newRightIndex = graphIndicesHolder.getMaxIndex();
                newLeftIndex = newRightIndex - (clickedRightIndex - clickedLeftIndex);
            }

            // System.out.println("x: " + draggedX + "newLeft: " + newLeftIndex + "newRight: " + newRightIndex);

            graphIndicesHolder.setLeftPlotIndex(newLeftIndex);
            graphIndicesHolder.setRightPlotIndex(newRightIndex);

            // graphArrayPanel.repaintGraphs();
        }
    }

    public static int getActionPerformedByDragAndDrop() {
        return actionPerformedByDragAndDrop;
    }

    public static void setActionPerformedByDragAndDrop(int actionPerformedByDragAndDrop) {
        YoGraph.actionPerformedByDragAndDrop = actionPerformedByDragAndDrop;
    }

    public static Object getSourceOfDrag() {
        return sourceOfDrag;
    }

    public static void setSourceOfDrag(Object sourceOfDrag) {
        YoGraph.sourceOfDrag = sourceOfDrag;
    }

    public static Object getRecipientOfDrag() {
        return recipientOfDrag;
    }

    public static void setRecipientOfDrag(Object recipientOfDrag) {
        YoGraph.recipientOfDrag = recipientOfDrag;
    }

    private void formatDouble(StringBuffer stringBuffer, double doubleValue) {
        doubleFormat.format(doubleValue, stringBuffer, fieldPosition); // Add the variable value to it
    }

    private void handleMouseEvent(javafx.scene.input.MouseEvent mevt) {
        if (mevt.getEventType().equals(javafx.scene.input.MouseEvent.MOUSE_PRESSED)) {
            this.handleMousePressed(mevt);
        } else if (mevt.getEventType().equals(javafx.scene.input.MouseEvent.MOUSE_DRAGGED)) {
            this.handleMouseDragged(mevt);
        }
    }

    private void handleKeyEvent(javafx.scene.input.KeyEvent kevt) {
        if (kevt.getEventType().equals(javafx.scene.input.KeyEvent.KEY_TYPED)) {
            this.handleKeyTyped(kevt);
        } else if (kevt.getEventType().equals(javafx.scene.input.KeyEvent.KEY_PRESSED)) {
            this.handleKeyPressed(kevt);
        } else if (kevt.getEventType().equals(javafx.scene.input.KeyEvent.KEY_RELEASED)) {
            this.handleKeyReleased(kevt);
        }
    }

    private void handleActionEvent(ActionEvent aevt) {
        if (aevt.getSource().equals(delete)) {
            setVisible(false);
            popupMenu.hide();
            this.yoGraphRemover.removeGraph(this);
        }
    }

    @Override
    public void handle(Event event) {
        if (event instanceof javafx.scene.input.MouseEvent) {
            this.handleMouseEvent((javafx.scene.input.MouseEvent) event);
        } else if (event instanceof javafx.scene.input.KeyEvent) {
            this.handleKeyEvent((javafx.scene.input.KeyEvent) event);
        } else if (event instanceof ActionEvent) {
            this.handleActionEvent((ActionEvent) event);
        }
    }
}
