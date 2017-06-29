package us.ihmc.simulationconstructionset.gui;

import javafx.application.Platform;
import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.event.ActionEvent;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.ContextMenu;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Polyline;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.GraphConfigurationChangeListener;
import us.ihmc.simulationconstructionset.gui.dialogs.GraphPropertiesDialog;
import us.ihmc.yoVariables.dataBuffer.*;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Graphical (JavaFX) display for up to a maximum number of <code>DataEntry</code> objects.
 * <p>
 * YoGraphs are optimized JavaFX-based displays to plot and navigate through DataEntry
 * objects. Data can be plotted against time or as phase plots between variables. Which
 * entries are on the graph is determined by the GUI configuration on startup and
 * can then be manipulated by drag-and-drop additions and on-click changes.
 * <p>
 * A YoGraph will occupy its containing space and handles drawing information on three
 * <code>Canvas</code> children:
 * <ul>
 * <li><strong>Top Layer:</strong> graph indices</li>
 * <li><strong>Mid Layer:</strong> baselines and labels</li>
 * <li><strong>Low Layer:</strong> entry data</li>
 * </ul>
 * <p>
 * Graph data can be scaled in multiple ways:
 * <table>
 * <tr>
 * <td></td>
 * <td><strong>Automatically</strong></td>
 * <td><strong>Manually</strong></td>
 * </tr>
 * <tr>
 * <td><strong>Globally</strong></td>
 * <td>Use entries' highest maximum and lowest minimum for all entries</td>
 * <td>Use graph's manually set minimum and maximum for all entries</td>
 * </tr>
 * <tr>
 * <td><strong>Individually</strong></td>
 * <td>Use entry's calculated minimum and maximum for each entry</td>
 * <td>Use entry's manually set minimum and maximum for each entry</td>
 * </tr>
 * </table>
 * Graph data is scaled upon each repaint.
 */
public class YoGraph extends Pane
      implements EventHandler<Event>, GraphConfigurationChangeListener, DataEntryChangeListener, us.ihmc.yoVariables.dataBuffer.DataBufferChangeListener,
      IndexChangedListener
{
   private static int DEFAULT_MAX_ENTRIES = 4;

   private static Color[] dataColors = {Color.rgb(0xa0, 0, 0), Color.rgb(0, 0, 0xff), Color.rgb(0, 0x80, 0), Color.rgb(0, 0, 0), Color.rgb(0x80, 0x80, 0x80),
         Color.rgb(0x80, 0, 0x80), Color.rgb(0, 0x80, 0x80), Color.rgb(0x60, 0x60, 0), Color.rgb(0xff, 0x50, 0x50), Color.rgb(0x50, 0xff, 0xff)};

   private static Color[] baselineColors = {Color.rgb(0x93, 0x70, 0xDB), Color.rgb(0x3C, 0xB3, 0x71), Color.ORANGE, Color.ORANGE, Color.ORANGE, Color.ORANGE};

   private static final int DONT_PLOT_BOTTOM_PIXELS = 25;
   private static final int PIXELS_PER_BOTTOM_ROW = 14;
   private static final int DONT_PLOT_TIMELINE_BOTTOM_PIXELS = 16;

   private GraphConfiguration graphConfiguration = new GraphConfiguration("default");

   private final JFrame parentFrame;

   private final TimeDataHolder timeDataHolder;
   private final DataEntryHolder dataEntryHolder;
   private final GraphIndicesHolder graphIndicesHolder;
   private final YoGraphRemover yoGraphRemover;

   public final static int MAX_NUM_GRAPHS = 10;
   public final static int MAX_NUM_BASELINES = 6;
   public final static int VAR_NAME_SPACING_FOR_PRINT = 160;

   private final ArrayList<DataEntry> entries;
   private final SelectedVariableHolder selectedVariableHolder;

   private double min = 0.0, max = 1.1;

   private final ArrayList<Integer> entryNamePaintWidths = new ArrayList<>();
   private final ArrayList<Integer> entryNamePaintRows = new ArrayList<>();
   private int totalEntryNamePaintRows = 1;
   private ContextMenu popupMenu;
   private javafx.scene.control.MenuItem delete;

   private static int actionPerformedByDragAndDrop = -1;
   private static Object sourceOfDrag = null;
   private static Object recipientOfDrag = null;

   private Canvas info = new Canvas(), index = new Canvas();

   private int focusedBaseLine = 0;

   public YoGraph(GraphIndicesHolder graphIndicesHolder, YoGraphRemover yoGraphRemover, SelectedVariableHolder holder, DataEntryHolder dataEntryHolder,
         TimeDataHolder timeDataHolder, JFrame jFrame)
   {
      this.selectedVariableHolder = holder;
      this.dataEntryHolder = dataEntryHolder;
      this.timeDataHolder = timeDataHolder;

      this.graphIndicesHolder = graphIndicesHolder;
      this.yoGraphRemover = yoGraphRemover;
      this.parentFrame = jFrame;

      this.entries = new ArrayList<>();

      info.setFocusTraversable(false);
      index.setFocusTraversable(false);

      this.graphConfiguration.addChangeListener(this);

      this.getChildren().addAll(info, index);

      this.widthProperty().addListener(observable ->  {
         this.resetCanvasSizes();
         this.repaintGraph(graphIndicesHolder.getLeftPlotIndex(), graphIndicesHolder.getRightPlotIndex());
         this.paintIndexLines();
      });
      this.heightProperty().addListener(observable -> {
         this.resetCanvasSizes();
         this.repaintGraph(graphIndicesHolder.getLeftPlotIndex(), graphIndicesHolder.getRightPlotIndex());
         this.paintIndexLines();
      });

      this.addEventHandler(javafx.scene.input.MouseEvent.ANY, this);
      this.addEventHandler(javafx.scene.input.KeyEvent.ANY, this);

      this.popupMenu = new ContextMenu();

      this.setOnContextMenuRequested(e ->
      {
         this.popupMenu.show(this, e.getScreenX(), e.getScreenY());
      });

      delete = new javafx.scene.control.MenuItem("Delete Graph");
      delete.addEventHandler(Event.ANY, this);
   }

   public GraphConfiguration getGraphConfiguration()
   {
      return this.graphConfiguration;
   }

   public void incrementBaseLine(int baseLineIndex, double scale)
   {
      if (baseLineIndex >= graphConfiguration.getBaselines().length)
         baseLineIndex = 0;

      double min = this.getMin();
      double max = this.getMax();

      double range = max - min;
      double amountToIncrement = 0.01 * range * scale;

      this.graphConfiguration.incrementBaseline(baseLineIndex, amountToIncrement);
   }

   /**
    * Changes the value of a baseline on this graph to be zero rather than its
    * currently calculated value.
    *
    * @param baseLineIndex     index of a baseline in this graph's baselines to be zeroed
    */
   public void zeroBaseLine(int baseLineIndex)
   {
      if (baseLineIndex >= graphConfiguration.getBaselines().length)
      {
         baseLineIndex = 0;
      }

      graphConfiguration.setBaseline(baseLineIndex, 0.0);
   }

   /**
    * Changes the value of a baseline on this graph to be the midpoint between
    * the current minimum and maximum rather than its currently calculated
    * value.
    *
    * @param baseLineIndex     index of a baseline in this graph's baselines to center
    */
   public void centerBaseLine(int baseLineIndex)
   {
      if (baseLineIndex >= graphConfiguration.getBaselines().length)
      {
         baseLineIndex = 0;
      }

      double min = this.getMin();
      double max = this.getMax();

      double center = (max + min) / 2.0;

      graphConfiguration.setBaseline(baseLineIndex, center);
   }

   public double[] getBaseLines()
   {
      return graphConfiguration.getBaselines();
   }

   /**
    * Calls <code>recalculateMinMax()</code> to find new maximum,
    * setting this graph's max and then returning it.
    *
    * @return newly calculated max
    */
   protected double getMax()
   {
      recalculateMinMax();

      return this.max;
   }

   /**
    * Calls <code>recalculateMinMax()</code> to find new minimum,
    * setting this graph's min and then returning it.
    *
    * @return newly calculated min
    */
   protected double getMin()
   {
      recalculateMinMax();

      return this.min;
   }

   protected void setGraphConfiguration(GraphConfiguration graphConfiguration)
   {
      if (graphConfiguration == null)
      {
         return;
      }

      this.graphConfiguration.setManualScalingMinMax(graphConfiguration.getManualScalingMin(), graphConfiguration.getManualScalingMax());
      this.graphConfiguration.setScaleType(graphConfiguration.getScaleType());
      this.graphConfiguration.setGraphType(graphConfiguration.getGraphType());
      this.graphConfiguration.setShowBaselines(graphConfiguration.getShowBaselines());
      this.graphConfiguration.setShowBaselinesInfo(graphConfiguration.getShowBaselinesInfo());
      this.graphConfiguration.setBaselines(graphConfiguration.getBaselines());
   }

   public ArrayList<DataEntry> getEntries()
   {
      return entries;
   }

   /**
    * Calls <code>isEmpty()</code> of this graph's entries to check
    * if there are any entries on the graph.
    *
    * @return whether or not there are entries on this graph
    */
   public boolean isEmpty()
   {
      return entries.isEmpty();
   }

   public void setInteractionEnable(boolean enable)
   {
      if (enable)
      {
         // First remove all listeners in case they're already there:
         // Could also use this.getListeners(MouseListener); and remove those attached...

         this.removeEventHandler(javafx.scene.input.MouseEvent.ANY, this);
         this.removeEventHandler(javafx.scene.input.KeyEvent.ANY, this);

         // Then add them back.

         this.addEventHandler(javafx.scene.input.MouseEvent.ANY, this);
         this.addEventHandler(javafx.scene.input.KeyEvent.ANY, this);
      }
      else
      {
         // Just disable them and assume no repeats...

         this.removeEventHandler(javafx.scene.input.MouseEvent.ANY, this);
         this.removeEventHandler(javafx.scene.input.KeyEvent.ANY, this);
      }
   }

   public int getNumVars()
   {
      return this.entries.size();
   }

   private double previousGraphWidth;

   private void calculateRequiredEntryPaintWidthsAndRows()
   {
      calculateRequiredEntryPaintWidthsAndRows(false);
   }

   private void calculateRequiredEntryPaintWidthsAndRows(boolean override)
   {
      double graphWidth = this.getWidth();

      if (graphWidth == this.previousGraphWidth && !override)
      {
         return;
      }

      this.entryNamePaintWidths.clear();
      this.entryNamePaintRows.clear();

      this.previousGraphWidth = graphWidth;

      int cumulatedWidth = 0;
      int row = 0;
      int numVars = this.entries.size();

      for (int i = 0; i < numVars; ++i)
      {
         DataEntry entry = this.entries.get(i);

         int variableWidth;

         Canvas forVar = (Canvas) this.getChildren().get(i * 2);
         GraphicsContext forCanvas = forVar.getGraphicsContext2D();

         int fontSize = (int) forCanvas.getFont().getSize();

         if (this.graphConfiguration.getShowNamespaces())
         {
            variableWidth = fontSize * entry.getVariable().getFullNameWithNameSpace().length();
         }
         else
         {
            variableWidth = fontSize * entry.getVariable().getName().length();
         }

         int variablePlusValueWidth = variableWidth + 120;

         if ((cumulatedWidth != 0) && (cumulatedWidth + variablePlusValueWidth > graphWidth))
         {
            row++;
            cumulatedWidth = 0;
         }

         cumulatedWidth += variablePlusValueWidth;

         this.entryNamePaintWidths.add(variablePlusValueWidth);

         this.entryNamePaintRows.add(row);
      }

      this.totalEntryNamePaintRows = row + 1;
   }

   public void addVariable(DataEntry entry)
   {
      if (entry == null)
      {
         return;
      }

      if (this.entries.size() >= YoGraph.MAX_NUM_GRAPHS)
      {
         return;
      }

      if (this.entries.contains(entry))
      {
         return;
      }

      this.entries.add(entry);

      entry.attachDataEntryChangeListener(this);

      this.getChildren().add(this.getChildren().size() - 2, new Canvas());
      this.getChildren().add(this.getChildren().size() - 2, new Canvas());

      this.recalculateMinMax();
      calculateRequiredEntryPaintWidthsAndRows();

      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex());
   }

   public void addVariableFromSelectedVariableHolder()
   {
      YoVariable<?> yoVariable = selectedVariableHolder.getSelectedVariable();

      if (yoVariable != null)
      {
         addVariable(dataEntryHolder.getEntry(yoVariable));
      }
   }

   public void removeEntry(DataEntry entry)
   {
      if (entries.contains(entry))
      {
         entry.detachDataEntryChangeListener(this);
         entries.remove(entry);
      }

      this.recalculateMinMax();
      calculateRequiredEntryPaintWidthsAndRows();
      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex());
   }

   private boolean hasMinMaxChanged()
   {
      if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.AUTO)
      {
         for (DataEntry entry : this.entries)
         {
            if (entry.hasMinMaxChanged())
            {
               if (entry.getMin() < this.min || entry.getMax() > this.max)
               {
                  return true;
               }
            }
         }
      }

      return false;
   }

   private void recalculateMinMax()
   {
      int numVars = this.entries.size();

      if (numVars < 1)
      {
         return;
      }

      double newMin = Double.POSITIVE_INFINITY, newMax = Double.NEGATIVE_INFINITY;

      for (DataEntry entry : this.entries)
      {
         boolean inverted = entry.getInverted();

         double entryMin = entry.getMin();
         double entryMax = entry.getMax();

         if (inverted)
         {
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

   private void calculateTimePlotData(DataEntry entry, double[] xData, double[] yData, int points, int beginningAt)
   {
      double width = this.getWidth();
      double height = this.getHeight() - totalDontPlotBottomPixels;

      int left = graphIndicesHolder.getLeftPlotIndex();
      int right = graphIndicesHolder.getRightPlotIndex();

      double[] realData = entry.getData(beginningAt, beginningAt + points);

      double min = this.minFor(entry);
      double max = this.maxFor(entry);

      double scaleX = width / (right - left);
      double scaleY = height / (max - min);

      for (int i = 0; i < points; ++i)
      {
         xData[i] = snap((beginningAt - left + i) * scaleX);
         yData[i] = snap(height - (realData[i] - min) * scaleY);
      }
   }

   private void calcScatterData(DataEntry entryX, DataEntry entryY, int nPoints, double[] xData, double[] yData, double minX, double maxX, double minY,
         double maxY, double width, double height, int offsetFromLeft, int offsetFromTop)
   {
      double[] dataX = entryX.getData();
      double[] dataY = entryY.getData();

      for (int i = 0; i < nPoints; i++)
      {
         xData[i] = ((dataX[i] - minX) / (maxX - minX) * width) + offsetFromLeft;
         yData[i] = height - (int) ((dataY[i] - minY) / (maxY - minY) * height) + offsetFromTop;
      }
   }

   private double[] zip(double[] a, double[] b)
   {
      if (a.length != b.length)
      {
         return new double[0];
      }

      double[] c = new double[a.length + b.length];

      for (int i = 0; i < a.length; ++i)
      {
         c[i * 2] = a[i];
         c[(i * 2) + 1] = b[i];
      }

      return c;
   }

   protected synchronized void printGraph(int printWidth, int printHeight)
   {
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      int numVars = entries.size();
      if (numVars == 0)
      {
         return;
      }

      int cumOffset = 3;

      // Here we use one scale for all the graphs:
      this.recalculateMinMax();

      for (int i = 0; i < numVars; i++)
      {
         cumOffset = i * ((int) (VAR_NAME_SPACING_FOR_PRINT * 0.6)) + 3;

         DataEntry entry = entries.get(i);
         double[] data = entry.getData();

         double minVal = 0.0, maxVal = 1.0;
         if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.INDIVIDUAL)
         {
            if (entry.isAutoScaleEnabled())
            {
               minVal = entry.getMin();
               maxVal = entry.getMax();
            }
            else
            {
               minVal = entry.getManualMinScaling();
               maxVal = entry.getManualMaxScaling();
            }
         }
         else if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.AUTO)
         {
            minVal = this.min;
            maxVal = this.max;
         }
         else if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.MANUAL)
         {
            minVal = this.graphConfiguration.getManualScalingMin();
            maxVal = this.graphConfiguration.getManualScalingMax();
         }

         int nPoints = data.length;

         int length;
         length = (length = ((outPoint - inPoint + 1 + nPoints) % nPoints)) == 0 ? nPoints : length;

         double[] xDataPrint = new double[length];
         double[] yDataPrint = new double[length];

         for (int j = 0; j < length; j++)
         {
            int index = (inPoint + j) % nPoints;
            xDataPrint[j] = (j * printWidth) / length;
            yDataPrint[j] =
                  (printHeight - DONT_PLOT_BOTTOM_PIXELS) - (int) ((data[index] - minVal) / (maxVal - minVal) * (printHeight - DONT_PLOT_BOTTOM_PIXELS));
         }

         Polyline thisPoly = new Polyline(zip(xDataPrint, yDataPrint));
         thisPoly.setStroke(dataColors[i % YoGraph.MAX_NUM_GRAPHS]);
         this.getChildren().add(thisPoly);
      }
   }

   private void resetCanvasSizes()
   {
      for (Node n : this.getChildren())
      {
         ((Canvas) n).setHeight(this.getHeight());
         ((Canvas) n).setWidth(this.getWidth());
      }
   }

   public void repaintGraph(int requestedLeftIndex, int requestedRightIndex)
   {
      this.repaintGraph(requestedLeftIndex, requestedRightIndex, false);
   }

   public void repaintGraph(int requestLeftIndex, int requestRightIndex, boolean override)
   {
      switch (this.graphConfiguration.getGraphType())
      {
      default:
      case TIME:
      {
         if (this.hasMinMaxChanged() || override)
         {
            this.recalculateMinMax();
            this.paintTimePlot(graphIndicesHolder.getLeftPlotIndex(), graphIndicesHolder.getRightPlotIndex());
         }
         else
         {
            for (DataEntry entry : this.entries)
            {
               if (entry.hasMinMaxChanged())
               {
                  entry.resetMinMaxChanged();
                  this.paintTimeEntryData(entry, requestLeftIndex, requestRightIndex - requestLeftIndex);
               }
            }
         }
         break;
      }
      case PHASE:
      {
         this.paintPhasePlot();
         break;
      }
      }

      this.paintVariableNamesAndValues();
   }

   private StringBuffer stringBuffer = new StringBuffer(80);
   @SuppressWarnings("unused") private String spaceString = "  ";
   private char[] charArray = new char[80];
   private final java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");
   private final FieldPosition fieldPosition = new FieldPosition(NumberFormat.INTEGER_FIELD);

   public void createBodePlotFromEntriesBetweenInOutPoints()
   {
      if (entries.size() < 2)
      {
         System.out.println("need 2 entries (input/output) for Bode plot");
         return;
      }
      if (!checkInOutPoints())
         return;

      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      DataEntry input = entries.get(0);
      DataEntry output = entries.get(1);

      double[] inputData = Arrays.copyOfRange(input.getData(), inPoint, outPoint);
      double[] outputData = Arrays.copyOfRange(output.getData(), inPoint, outPoint);

      double[] timeData = Arrays.copyOfRange(timeDataHolder.getTimeData(), inPoint, outPoint);

      BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
   }

   public void createBodePlotFromEntries()
   {
      if (entries.size() < 2)
         return;

      DataEntry input = entries.get(0);
      DataEntry output = entries.get(1);

      double[] inputData = input.getData();
      double[] outputData = output.getData();

      double[] timeData = timeDataHolder.getTimeData();

      BodePlotConstructor.plotBodeFromInputToOutput(input.getVariableName(), output.getVariableName(), timeData, inputData, outputData);
   }

   private boolean checkInOutPoints()
   {
      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      boolean valid = outPoint > inPoint;

      if (!valid)
      {
         System.out.println("Please set inPoint < outPoint and re-try");
      }

      return valid;
   }

   public void createFFTPlotsFromEntriesBetweenInOutPoints()
   {
      if (!checkInOutPoints())
         return;

      int inPoint = graphIndicesHolder.getInPoint();
      int outPoint = graphIndicesHolder.getOutPoint();

      double[] timeData = timeDataHolder.getTimeData();
      double[] rngTimeData = Arrays.copyOfRange(timeData, inPoint, outPoint);

      for (DataEntry entry : entries)
      {
         double[] data = entry.getData();
         double[] rngData = Arrays.copyOfRange(data, inPoint, outPoint);

         BodePlotConstructor.plotFFT(entry.getVariableName(), rngTimeData, rngData);
      }
   }

   public void createFFTPlotsFromEntries()
   {
      double[] timeData = timeDataHolder.getTimeData();

      for (DataEntry entry : entries)
      {
         double[] data = entry.getData();

         BodePlotConstructor.plotFFT(entry.getVariableName(), timeData, data);
      }
   }

   public double minFor(DataEntry entry)
   {
      if (!this.entries.contains(entry))
      {
         throw new NullPointerException("entry does not exist on this graph");
      }

      if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.INDIVIDUAL)
      {
         return entry.getMin();
      }
      else
      {
         if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.AUTO)
         {
            this.recalculateMinMax();
         }

         return this.min;
      }
   }

   public double maxFor(DataEntry entry)
   {
      if (!this.entries.contains(entry))
      {
         throw new NullPointerException("entry does not exist on this graph");
      }

      if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.INDIVIDUAL)
      {
         return entry.getMax();
      }
      else
      {
         if (this.graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.AUTO)
         {
            this.recalculateMinMax();
         }

         return this.max;
      }
   }

   public void paintPhasePlot()
   {
      double graphWidth = this.getWidth();
      double graphHeight = this.getHeight();

      int numVars = entries.size();

      GraphicsContext dataLayer = ((Canvas) this.getChildren().get(0)).getGraphicsContext2D();

      for (int i = 0; i < numVars / 2; i++)
      {
         DataEntry entryX = entries.get(i);
         double[] dataX = entryX.getData();

         DataEntry entryY = entries.get(i + 1);

         double minValX = 0.0, maxValX = 1.0;
         double minValY = 0.0, maxValY = 1.0;

         if (graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.INDIVIDUAL)
         {
            if (entryX.isAutoScaleEnabled())
            {
               minValX = entryX.getMin();
               maxValX = entryX.getMax();
            }
            else
            {
               minValX = entryX.getManualMinScaling();
               maxValX = entryX.getManualMaxScaling();
            }

            if (entryY.isAutoScaleEnabled())
            {
               minValY = entryY.getMin();
               maxValY = entryY.getMax();
            }
            else
            {
               minValY = entryY.getManualMinScaling();
               maxValY = entryY.getManualMaxScaling();
            }
         }
         else if (graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.AUTO)
         {
            // minValX = minValY = this.min;   //++++++
            // maxValX = maxValY = this.max;

            minValY = entryY.getMin();
            maxValY = entryY.getMax();

            minValX = entryX.getMin();
            maxValX = entryX.getMax();
         }
         else if (graphConfiguration.getScaleType() == GraphConfiguration.ScaleType.MANUAL)
         {
            minValX = minValY = graphConfiguration.getManualScalingMin();    // ++++++
            maxValX = maxValY = graphConfiguration.getManualScalingMax();
         }

         int nPoints = dataX.length;

         double[] xData = new double[nPoints], yData = new double[nPoints];

         int totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (totalEntryNamePaintRows - 1);

         calcScatterData(entryX, entryY, nPoints, xData, yData, minValX, maxValX, minValY, maxValY, (graphWidth - 6), graphHeight - totalDontPlotBottomPixels,
               3, 5);

         dataLayer.setStroke(dataColors[i % YoGraph.MAX_NUM_GRAPHS]);
         dataLayer.strokePolyline(xData, yData, xData.length);

         // Draw a Cross Hairs:
         int index = graphIndicesHolder.getIndex();

         if ((index < xData.length) && (index < yData.length) & (index >= 0))
         {
            dataLayer.setStroke(Color.BLACK);
            dataLayer.strokeLine(xData[index] - 5, yData[index], xData[index] + 5, yData[index]);
            dataLayer.strokeLine(xData[index], yData[index] - 10, xData[index], yData[index] + 10);
         }
      }

      paintVariableNamesAndValues();
   }

   private double snap(double value)
   {
      return 0.5 + (int) value;
   }

   public void paintTimePlot(int requestLeftIndex, int requestRightIndex)
   {
      for (DataEntry entry : this.entries)
      {
         this.paintTimeEntryData(entry, requestLeftIndex, requestRightIndex - requestLeftIndex);
      }
   }

   public void paintTimeEntryData(DataEntry entry, int requestLeftIndex, int requestPoints)
   {
      if (requestPoints < 2)
         return;

      final int points = Math.min(entry.getData().length, requestPoints);

      int index = this.entries.indexOf(entry);

      if (index == -1)
         return;

      Platform.runLater(() ->
      {
         double height = this.getHeight();
         double width = this.getWidth();

         Canvas forData = (Canvas) this.getChildren().get(2 * index);

         forData.setWidth(width);
         forData.setHeight(height);

         GraphicsContext dataLayer = forData.getGraphicsContext2D();

         double[] xData = new double[points];
         double[] yData = new double[points];

         calculateTimePlotData(entry, xData, yData, points, requestLeftIndex);

         dataLayer.clearRect(snap(Math.ceil(xData[0])), 0, snap(Math.floor(xData[points - 1] - xData[0])), height);

         dataLayer.setStroke(dataColors[index]);

         dataLayer.strokePolyline(xData, yData, points);

         if (this.graphConfiguration.getShowBaselines())
         {
            Canvas forBase = (Canvas) this.getChildren().get((2 * index) + 1);

            forBase.setWidth(width);
            forBase.setHeight(height);

            GraphicsContext baseLayer = forBase.getGraphicsContext2D();

            baseLayer.clearRect(0, 0, width, height);

            double[] baselines = this.graphConfiguration.getBaselines();

            for (int j = 0; j < baselines.length; j++)
            {
               double entryMin = this.minFor(entry);

               double baseY = height - this.totalDontPlotBottomPixels - (baselines[j] - entryMin) / (this.maxFor(entry) - entryMin) * (height
                     - this.totalDontPlotBottomPixels) + 5;

               baseLayer.setStroke(baselineColors[j]);
               baseLayer.strokeLine(0, baseY, this.getWidth(), baseY);
            }
         }

         paintVariableNamesAndValues();
      });
   }

   public void paintIndexLines()
   {
      Platform.runLater(() ->
      {
         double width = this.getWidth();
         double height = this.getHeight() - totalDontPlotBottomPixels;
         int inPoint = this.graphIndicesHolder.getInPoint();
         int outPoint = this.graphIndicesHolder.getOutPoint();
         int leftIndex = this.graphIndicesHolder.getLeftPlotIndex();
         int rightIndex = this.graphIndicesHolder.getRightPlotIndex();

         index.setWidth(width);
         index.setHeight(height);

         GraphicsContext gc = index.getGraphicsContext2D();

         gc.clearRect(0, 0, width, height);
         gc.setLineWidth(1.5d);

         double linex;

         if (inPoint >= leftIndex)
         {
            linex = snap(((inPoint - leftIndex) * width) / (rightIndex - leftIndex));
            gc.setStroke(Color.GREEN);
            gc.strokeLine(linex, 0, linex, height);
         }

         if (outPoint <= rightIndex)
         {
            linex = snap(((outPoint - leftIndex) * width) / (rightIndex - leftIndex));
            gc.setStroke(Color.RED);
            gc.strokeLine(linex, 0, linex, height);
         }

         {
            ArrayList<Integer> keys = graphIndicesHolder.getKeyPoints();

            for (Integer key : keys)
            {
               if (key >= leftIndex && key <= rightIndex)
               {
                  linex = snap((key * width) / (rightIndex - leftIndex));
                  gc.setStroke(Color.ORANGE);
                  gc.strokeLine(linex, 0, linex, height);
               }
            }
         }

         {
            linex = snap(((this.graphIndicesHolder.getIndex() - leftIndex) * width) / (rightIndex - leftIndex));
            gc.setStroke(Color.BLACK);
            gc.strokeLine(linex, 0, linex, height);
         }
      });
   }

   private void paintVariableNamesAndValues()
   {
      Platform.runLater(() ->
      {
         calculateRequiredEntryPaintWidthsAndRows(true);
         this.totalDontPlotBottomPixels = DONT_PLOT_BOTTOM_PIXELS + PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - 1);

         double height = this.getHeight();

         GraphicsContext baselineLayer = info.getGraphicsContext2D();

         baselineLayer.clearRect(0, height - this.totalDontPlotBottomPixels, this.getWidth(), this.totalDontPlotBottomPixels);

         drawBaselineInfo();

         if (this.graphConfiguration.getShowBaselinesInfo())
         {
            return;
         }

         baselineLayer.setFill(Color.BLACK);
         baselineLayer.setLineWidth(1.5f);

         int numVars = this.entries.size();

         int previousRow = 0;
         int cumulativeOffset = 3;

         for (int i = 0; i < numVars; i++)
         {
            DataEntry entry = this.entries.get(i);

            if (this.graphConfiguration.getGraphType() == GraphConfiguration.GraphType.PHASE)
            {
               baselineLayer.setFill(dataColors[i / 2 % DEFAULT_MAX_ENTRIES]);
            }
            else
            {
               baselineLayer.setFill(dataColors[i % DEFAULT_MAX_ENTRIES]);
            }

            // Draw the variable name
            int row = entryNamePaintRows.get(i);
            if (row != previousRow)
            {
               cumulativeOffset = 3;
               previousRow = row;
            }

            String display = "";

            if (this.graphConfiguration.getShowNamespaces())
            {
               display = entry.getVariable().getNameSpace() + ".";
            }

            display += entry.getVariable().getName();

            if (this.graphIndicesHolder.getIndex() < this.graphIndicesHolder.getOutPoint())
            {
               display += String.format(": %.4f", entry.getData()[this.graphIndicesHolder.getIndex()]);
            }
            else if (this.graphIndicesHolder.getOutPoint() > 0)
            {
               display += String.format(": %.4f", entry.getData()[this.graphIndicesHolder.getOutPoint() - 1]);
            }

            int yToDrawAt = (int) height - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - row - 1));

            baselineLayer.fillText(display, cumulativeOffset, yToDrawAt);

            cumulativeOffset += entryNamePaintWidths.get(i);
         }
      });
   }

   private void drawBaselineInfo()
   {
      Platform.runLater(() ->
      {
         double graphHeight = this.getHeight();
         int yToDrawAt = (int) graphHeight - 5 - (PIXELS_PER_BOTTOM_ROW * (this.totalEntryNamePaintRows - 1));

         double total = 0.0;

         GraphicsContext baselineLayer = info.getGraphicsContext2D();

         if (!this.graphConfiguration.getShowBaselinesInfo())
         {
            return;
         }

         double[] baselines = this.graphConfiguration.getBaselines();

         if (baselines.length == 0)
         {
            return;
         }

         int cumOffset = 3;

         baselineLayer.setFill(Color.BLACK);
         baselineLayer.fillText("Baselines: ", cumOffset, yToDrawAt);
         cumOffset += 80;

         baselineLayer.setLineWidth(1.5f);

         for (int i = 0; i < baselines.length; i++)
         {
            String display = "";
            double baseline = baselines[i];
            total = total + baseline;

            display += String.format("%.4f", baseline);

            int baseLineStringWidth = (int) baselineLayer.getFont().getSize() * display.length();

            baselineLayer.setFill(dataColors[i]);
            baselineLayer.fillText(display, cumOffset, yToDrawAt);
            cumOffset = cumOffset + baseLineStringWidth + 10;
         }

         double average = total / ((double) baselines.length);

         baselineLayer.setFill(Color.BLACK);

         baselineLayer.fillText(String.format("     Average = %.4f", average), cumOffset, yToDrawAt);
      });
   }

   public void handleKeyPressed(javafx.scene.input.KeyEvent evt)
   {
      KeyCode code = evt.getCode();

      switch (code)
      {
      case LEFT:
         this.graphIndicesHolder.tickLater(-1);
         break;
      case RIGHT:
         this.graphIndicesHolder.tickLater(1);
         break;
      case ALT:
         this.graphConfiguration.setShowNamespaces(true);
         break;
      case CONTROL:
         this.graphConfiguration.setShowBaselinesInfo(true);
         break;
      case UP:
         incrementBaseLine(focusedBaseLine, 1.0);
         break;
      case DOWN:
         incrementBaseLine(focusedBaseLine, -1.0);
         break;
      }
   }

   public void handleKeyReleased(javafx.scene.input.KeyEvent evt)
   {
      KeyCode code = evt.getCode();

      switch (code)
      {
      case ALT:
      {
         this.graphConfiguration.setShowNamespaces(false);
         break;
      }
      case CONTROL:
      {
         this.graphConfiguration.setShowBaselinesInfo(false);
         break;
      }
      }
   }

   public void handleKeyTyped(javafx.scene.input.KeyEvent evt)
   {
      String character = evt.getCharacter();

      switch (character)
      {
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

   private void handleMousePressed(javafx.scene.input.MouseEvent evt)
   {
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
      this.clickedIndex = clickIndex(x);
      this.clickedLeftIndex = graphIndicesHolder.getLeftPlotIndex();
      this.clickedRightIndex = graphIndicesHolder.getRightPlotIndex();

      // Double click brings up var properties dialog box.
      if ((evt.getClickCount() == 2) && (!entries.isEmpty()))
      {
         if (parentFrame != null)
         {
            GraphPropertiesDialog dialog = new GraphPropertiesDialog(parentFrame, this);
            dialog.show();
         }
      }

      // Right click places and deletes graphs:

      // if (evt.isControlDown())
      // if (evt.isMetaDown() && evt.isAltDown())
      // if (evt.isMetaDown() && evt.isControlDown())
      // if (evt.isShiftDown())
      if (evt.isMiddleButtonDown())
      {    // Middle Click
         // If mouse was pressed in a label, remove that variable:

         if (y > h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)
         {
            int idx = getClickedVariableIndex(x, y, h);

            if (idx < this.entries.size())
            {
               this.removeEntry(this.entries.get(idx));
            }
         }
         else
         {
            addVariableFromSelectedVariableHolder();
         }
      }
      else if (evt.isPrimaryButtonDown())
      {
         if ((this.entries == null) || (this.entries.size() < 1) || (this.graphConfiguration.getGraphType() == GraphConfiguration.GraphType.PHASE))
         {
            return;
         }

         if (y <= h - this.totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)
         {
            int newIndex = clickIndex(x);

            this.graphIndicesHolder.setIndexLater(newIndex);
         }
         // If mouse was pressed in a label, highlight that variable and initiate drag and drop:
         else
         {
            int index = getClickedVariableIndex(x, y, h);

            if (index < this.entries.size())
            {
               DataEntry entry = entries.get(index);
               selectedVariableHolder.setSelectedVariable(entry.getVariable());

               if (!evt.isControlDown())
               {
                  //this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.MOVE);
                  actionPerformedByDragAndDrop = TransferHandler.MOVE;
               }
               else if (evt.isControlDown())
               {
                  //this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.COPY);
                  actionPerformedByDragAndDrop = TransferHandler.COPY;
               }

               sourceOfDrag = this;
            }
         }

      }
      else if (evt.isSecondaryButtonDown())
      {
         popupMenu.setAnchorX(evt.getScreenX());
         popupMenu.setAnchorY(evt.getScreenY());

         popupMenu.getItems().clear();

         for (final DataEntry dataBufferEntry : this.entries)
         {
            final javafx.scene.control.MenuItem menuItem = new javafx.scene.control.MenuItem("Remove " + dataBufferEntry.getVariableName());

            menuItem.addEventHandler(ActionEvent.ANY, e ->
            {
               removeEntry(dataBufferEntry);
               this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex());
               popupMenu.getItems().remove(menuItem);
               popupMenu.hide();
            });

            popupMenu.getItems().add(menuItem);
         }

         popupMenu.getItems().add(delete);
      }
   }

   private int getClickedVariableIndex(double x, double y, double graphHeight)
   {
      int rowClicked = (int) (y - (graphHeight - totalEntryNamePaintRows * PIXELS_PER_BOTTOM_ROW)) / PIXELS_PER_BOTTOM_ROW;

      int index = 0;
      int totalOffset = 3;

      for (int j = 0; j < entryNamePaintWidths.size(); j++)
      {
         Integer entryPaintWidth = entryNamePaintWidths.get(j);
         Integer row = entryNamePaintRows.get(j);

         if (row < rowClicked)
         {
            index++;
         }
         else
         {
            totalOffset = totalOffset + entryPaintWidth;

            if (x > totalOffset)
            {
               index++;
            }
         }
      }

      return index;
   }

   private int clickIndex(double mouseX)
   {
      int left = this.graphIndicesHolder.getLeftPlotIndex();

      return left + (int) Math.floor(mouseX / this.getWidth() * (this.graphIndicesHolder.getRightPlotIndex() - left));
   }

   @SuppressWarnings("unused") private double clickedX, clickedY;
   @SuppressWarnings("unused") private double draggedX, draggedY;
   private int clickedIndex, clickedLeftIndex, clickedRightIndex;

   public void handleMouseDragged(javafx.scene.input.MouseEvent evt)
   {
      draggedX = evt.getX();
      draggedY = evt.getY();

      double h = getHeight();
      double w = getWidth();

      if (draggedX > w)
      {
         draggedX = w;
      }

      if (draggedX < 0)
      {
         draggedX = 0;
      }

      if (clickedY > h - DONT_PLOT_TIMELINE_BOTTOM_PIXELS)
      {
         return;
      }

      if (evt.isPrimaryButtonDown() && (this.graphConfiguration.getGraphType() != GraphConfiguration.GraphType.PHASE))
      {
         int index = clickIndex(draggedX);
         graphIndicesHolder.setIndexLater(index);    // +++JEP setIndex or setIndexLater??

         // graphArrayPanel.setIndex(index); //+++JEP setIndex or setIndexLater??
         // clickIndex = index;

         // System.out.println("x: " + draggedX + "index: " + index);
      }

      if (evt.isSecondaryButtonDown())
      {
         int index = clickIndex(draggedX);

         int newLeftIndex = clickedLeftIndex + clickedIndex - index;
         int newRightIndex = clickedRightIndex + clickedIndex - index;

         if (newLeftIndex < 0)
         {
            newLeftIndex = 0;
            newRightIndex = clickedRightIndex - clickedLeftIndex;
         }

         if (newRightIndex > graphIndicesHolder.getMaxIndex())
         {
            newRightIndex = graphIndicesHolder.getMaxIndex();
            newLeftIndex = newRightIndex - (clickedRightIndex - clickedLeftIndex);
         }

         graphIndicesHolder.setLeftPlotIndex(newLeftIndex);
         graphIndicesHolder.setRightPlotIndex(newRightIndex);
      }
   }

   public static int getActionPerformedByDragAndDrop()
   {
      return actionPerformedByDragAndDrop;
   }

   public static void setActionPerformedByDragAndDrop(int actionPerformedByDragAndDrop)
   {
      YoGraph.actionPerformedByDragAndDrop = actionPerformedByDragAndDrop;
   }

   public static Object getSourceOfDrag()
   {
      return sourceOfDrag;
   }

   public static void setSourceOfDrag(Object sourceOfDrag)
   {
      YoGraph.sourceOfDrag = sourceOfDrag;
   }

   public static Object getRecipientOfDrag()
   {
      return recipientOfDrag;
   }

   public static void setRecipientOfDrag(Object recipientOfDrag)
   {
      YoGraph.recipientOfDrag = recipientOfDrag;
   }

   private void handleMouseEvent(javafx.scene.input.MouseEvent mevt)
   {
      if (mevt.getEventType().equals(javafx.scene.input.MouseEvent.MOUSE_PRESSED))
      {
         this.handleMousePressed(mevt);
      }
      else if (mevt.getEventType().equals(javafx.scene.input.MouseEvent.MOUSE_DRAGGED))
      {
         this.handleMouseDragged(mevt);
      }
   }

   private void handleKeyEvent(KeyEvent kevt)
   {
      if (kevt.getEventType().equals(KeyEvent.KEY_TYPED))
      {
         this.handleKeyTyped(kevt);
      }
      else if (kevt.getEventType().equals(KeyEvent.KEY_PRESSED))
      {
         this.handleKeyPressed(kevt);
      }
      else if (kevt.getEventType().equals(KeyEvent.KEY_RELEASED))
      {
         this.handleKeyReleased(kevt);
      }
   }

   private void handleActionEvent(ActionEvent aevt)
   {
      if (aevt.getSource().equals(delete))
      {
         setVisible(false);
         popupMenu.hide();
         this.yoGraphRemover.removeGraph(this);
      }
   }

   @Override public void handle(Event event)
   {
      if (event instanceof javafx.scene.input.MouseEvent)
      {
         this.handleMouseEvent((javafx.scene.input.MouseEvent) event);
      }
      else if (event instanceof javafx.scene.input.KeyEvent)
      {
         this.handleKeyEvent((javafx.scene.input.KeyEvent) event);
      }
      else if (event instanceof ActionEvent)
      {
         this.handleActionEvent((ActionEvent) event);
      }
   }

   @Override public void notifyOfGraphTypeChange()
   {
      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex(), true);
   }

   @Override public void notifyOfBaselineChange()
   {
      // do nothing; if baselines changed, data changed too, and baselines will be redrawn anyway
   }

   @Override public void notifyOfScaleChange()
   {
      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex(), true);
   }

   @Override public void notifyOfDisplayChange()
   {
      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex(), true);
   }

   @Override public void notifyOfDataChange(DataEntry entry, int index)
   {
      if (this.graphConfiguration.getGraphType() == GraphConfiguration.GraphType.TIME)
      {
         this.paintTimeEntryData(entry, Math.max(0, index - 1), 2);
      }
      else if (this.graphConfiguration.getGraphType() == GraphConfiguration.GraphType.PHASE)
      {
         this.paintPhasePlot();
      }
   }

   @Override public void notifyOfBufferChange()
   {
      this.repaintGraph(this.graphIndicesHolder.getLeftPlotIndex(), this.graphIndicesHolder.getRightPlotIndex(), true);
   }

   @Override public void wasRewound()
   {
      // do nothing; data isn't changing and paintIndexLines will be called anyway
   }

   @Override public void notifyOfIndexChange(int newIndex, double newTime)
   {
      this.paintVariableNamesAndValues();
      this.paintIndexLines();
   }
}
