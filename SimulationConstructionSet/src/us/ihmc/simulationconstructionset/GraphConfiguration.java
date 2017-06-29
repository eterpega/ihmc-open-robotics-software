package us.ihmc.simulationconstructionset;

import us.ihmc.simulationconstructionset.gui.YoGraph;
import us.ihmc.tools.io.xml.XMLReaderUtility;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.StringTokenizer;

public class GraphConfiguration
{
   public enum GraphType
   {
      TIME, PHASE
   }

   public enum ScaleType
   {
      AUTO, MANUAL, INDIVIDUAL
   }

   private final String name;
   private static int id = 1;

   private ArrayList<GraphConfigurationChangeListener> changeListeners = new ArrayList<>();

   // private String[] varNames;

   private double manualMinScaling = 0.0, manualMaxScaling = 1.0;

   // private final double manualMinScaling, manualMaxScaling;
   @SuppressWarnings("unused") private double minPhaseXScaling = 0.0, maxPhaseXScaling = 1.0;

   private ScaleType scaleType = ScaleType.AUTO;
   private GraphType graphType = GraphType.TIME;

   private boolean showBaselines = true;
   private boolean showBaselinesInfo = false;
   private boolean showNamespaces = false;
   private double[] baselines = new double[] {0};

   private static final GraphConfiguration standardAutoScalingConfiguration = new GraphConfiguration("auto", GraphConfiguration.ScaleType.AUTO);

   static
   {
      standardAutoScalingConfiguration.setBaseline(0.0);
      standardAutoScalingConfiguration.setShowBaselines(true);
   }

   public static GraphConfiguration getStandardAutoScalingConfiguration()
   {
      return standardAutoScalingConfiguration;
   }

   public GraphConfiguration(String name)
   {
      this.name = name;
   }

   public GraphConfiguration(String name, ScaleType scaleType)
   {
      this.name = name;
      this.scaleType = scaleType;
   }

   public GraphConfiguration(String name, ScaleType scaleType, double minScaling, double maxScaling)
   {
      this.name = name;
      this.scaleType = scaleType;
      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   public String getName()
   {
      return this.name;
   }

   public void setScaleType(ScaleType scaleType)
   {
      if (this.scaleType != scaleType)
      {
         this.scaleType = scaleType;

         this.notifyScaleChangeListeners();
      }
   }

   public ScaleType getScaleType()
   {
      return this.scaleType;
   }

   public void setGraphType(GraphType graphType)
   {
      if (this.graphType != graphType)
      {
         this.graphType = graphType;

         this.notifyGraphTypeChangeListeners();
      }
   }

   public GraphType getGraphType()
   {
      return this.graphType;
   }

   public void setShowNamespaces(boolean showNamespaces)
   {
      if (this.showNamespaces != showNamespaces)
      {
         this.showNamespaces = showNamespaces;

         this.notifyDisplayChangeListeners();
      }
   }

   public void setShowBaselines(boolean showBaselines)
   {
      if (this.showBaselines != showBaselines)
      {
         this.showBaselines = showBaselines;

         this.notifyBaselineChangeListeners();
      }
   }

   public void setShowBaselinesInfo(boolean showBaselinesInfo)
   {
      if (this.showBaselinesInfo != showBaselinesInfo)
      {
         this.showBaselinesInfo = showBaselinesInfo;

         this.notifyDisplayChangeListeners();
      }
   }

   public void setBaseline(double baseline)
   {
      this.baselines = new double[] {baseline};

      this.notifyBaselineChangeListeners();
   }

   public void setBaselines(double... baselines)
   {
      if (baselines.length < YoGraph.MAX_NUM_BASELINES)
      {
         this.baselines = baselines;
      }
      else
      {
         this.baselines = Arrays.copyOfRange(baselines, 0, YoGraph.MAX_NUM_BASELINES);
      }

      this.notifyBaselineChangeListeners();
   }

   public void setPositiveNegativeBaselines(double baseline)
   {
      this.baselines = new double[] {-baseline, baseline};

      this.notifyBaselineChangeListeners();
   }

   public void setBaseline(int baselineIndex, double value)
   {
      if (baselineIndex >= baselines.length)
         return;

      this.baselines[baselineIndex] = value;

      this.notifyBaselineChangeListeners();
   }

   public void incrementBaseline(int baselineIndex, double amountToIncrement)
   {
      if (baselineIndex >= baselines.length)
         return;

      this.baselines[baselineIndex] += amountToIncrement;

      this.notifyBaselineChangeListeners();
   }

   public boolean getShowNamespaces()
   {
      return this.showNamespaces;
   }

   public boolean getShowBaselines()
   {
      return this.showBaselines;
   }

   public boolean getShowBaselinesInfo()
   {
      return this.showBaselinesInfo;
   }

   public double[] getBaselines()
   {
      return this.baselines;
   }

   public void setManualScalingMinMax(double minScaling, double maxScaling)
   {
      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
      this.notifyScaleChangeListeners();
   }

   public void setPhasePlotXScalingMinMax(double minPhaseXScaling, double maxPhaseXScaling)
   {
      this.minPhaseXScaling = minPhaseXScaling;
      this.maxPhaseXScaling = maxPhaseXScaling;
      this.notifyScaleChangeListeners();
   }

   public double getManualScalingMin()
   {
      return manualMinScaling;
   }

   public double getManualScalingMax()
   {
      return manualMaxScaling;
   }

   public void addChangeListener(GraphConfigurationChangeListener listener) {
      this.changeListeners.add(listener);
   }

   private void notifyGraphTypeChangeListeners()
   {
      for (GraphConfigurationChangeListener listener : this.changeListeners)
      {
         listener.notifyOfGraphTypeChange();
      }
   }

   private void notifyScaleChangeListeners()
   {
      for (GraphConfigurationChangeListener listener : this.changeListeners)
      {
         listener.notifyOfScaleChange();
      }
   }

   private void notifyBaselineChangeListeners()
   {
      for (GraphConfigurationChangeListener listener : this.changeListeners)
      {
         listener.notifyOfBaselineChange();
      }
   }

   private void notifyDisplayChangeListeners()
   {
      for (GraphConfigurationChangeListener listener : this.changeListeners)
      {
         listener.notifyOfDisplayChange();
      }
   }

   public String getXMLStyleRepresentationOfClass()
   {
      String returnString = "\t\t<GraphConfiguration>\n";

      returnString += "\t\t\t<Name>";
      returnString += "config" + id;
      id++;
      returnString += "</Name>\n";
      returnString += "\t\t\t<ScalingMethod>";
      returnString += scaleType.ordinal();
      returnString += "</ScalingMethod>\n";
      returnString += "\t\t\t<PlotType>";
      returnString += graphType.ordinal();
      returnString += "</PlotType>\n";
      returnString += "\t\t\t<ShowBaselines>";
      returnString += showBaselines;
      returnString += "</ShowBaselines>\n";
      returnString += "\t\t\t<Baselines>";

      if (baselines != null)
      {
         for (double baseline : baselines)
         {
            returnString += baseline + ",";
         }
      }

      returnString += "</Baselines>\n";
      returnString += "\t\t\t<MaxScaling>";
      returnString += manualMaxScaling;
      returnString += "</MaxScaling>\n";
      returnString += "\t\t\t<MinScaling>";
      returnString += manualMinScaling;
      returnString += "</MinScaling>\n";
      returnString += "\t\t</GraphConfig>";

      return returnString;
   }

   public static GraphConfiguration createClassBasedOnXMLRepresentation(int start, String xmlRepresentation)
   {
      GraphConfiguration tmp = null;
      try
      {
         String graphConfigurationString = XMLReaderUtility.getMiddleString(start, xmlRepresentation, "<GraphConfiguration>", "</GraphConfig>");

         //       System.out.println("        GraphConfiguration: " + graphConfigurationString);

         String name = XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<Name>", "</Name>");

         //       System.out.println("            Name: " + name);

         ScaleType scaleType = ScaleType.AUTO;
         int scaleTypeVal = XMLReaderUtility.parseIntegerBetweenTwoStrings(0, graphConfigurationString, "<ScalingMethod>",
               "</ScalingMethod>");    // Integer.parseInt(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<ScalingMethod>", "</ScalingMethod>"));

         if (scaleTypeVal >= 0 && scaleTypeVal < ScaleType.values().length) {
            scaleType = ScaleType.values()[scaleTypeVal];
         }

         //       System.out.println("            ScalingMethod: " + scaleType);

         GraphType graphType = GraphType.TIME;
         int graphTypeVal = XMLReaderUtility.parseIntegerBetweenTwoStrings(0, graphConfigurationString, "<PlotType>",
               "</PlotType>");    // Integer.parseInt(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<PlotType>", "</PlotType>"));

         if (graphTypeVal >= 0 && graphTypeVal < GraphType.values().length) {
            graphType = GraphType.values()[graphTypeVal];
         }

         //       System.out.println("            PlotType: " + graphType);

         boolean showBaselines = XMLReaderUtility.parseBooleanBetweenTwoStrings(0, graphConfigurationString, "<ShowBaselines>",
               "</ShowBaselines>");    // Boolean.parseBoolean(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<ShowBaselines>", "</ShowBaselines>"));

         //       System.out.println("            ShowBaselines: " + showBaselines);

         String baselinesString = XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<Baselines>", "</Baselines>");

         //       System.out.println("            Baselines: " + baselines);

         StringTokenizer tokenizer = new StringTokenizer(baselinesString, " /t/n/r/f,");
         double[] baselines = new double[tokenizer.countTokens()];
         int numberOfTokens = tokenizer.countTokens();
         for (int i = 0; i < numberOfTokens; i++)
         {
            baselines[i] = XMLReaderUtility.parseDouble(tokenizer.nextToken());    // Double.parseDouble(tokenizer.nextToken());

            //          System.out.println("                Baseline: " + baselines[i]);
         }

         double manualMaxScaling = XMLReaderUtility.parseDoubleBetweenTwoStrings(0, graphConfigurationString, "<MaxScaling>",
               "</MaxScaling>");    // Double.parseDouble(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<MaxScaling>", "</MaxScaling>"));

         //       System.out.println("            ManualMaxScaling: " + manualMaxScaling);

         double manualMinScaling = XMLReaderUtility.parseDoubleBetweenTwoStrings(0, graphConfigurationString, "<MinScaling>",
               "</MinScaling>");    // Double.parseDouble(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<MinScaling>", "</MinScaling>"));

         //       System.out.println("            ManualMinScaling: " + manualMinScaling);

         tmp = new GraphConfiguration(name, scaleType, manualMinScaling, manualMaxScaling);

         // this.name = name;
         // this.scaleType = scaleType;
         tmp.setGraphType(graphType);
         tmp.setShowBaselines(showBaselines);
         tmp.setBaselines(baselines);

         // this.manualMaxScaling = manualMaxScaling;
         // this.manualMinScaling = manualMinScaling;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         XMLReaderUtility.displayErrorMessage();

         return null;
      }

      return tmp;
   }
}
