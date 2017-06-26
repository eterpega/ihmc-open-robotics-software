package us.ihmc.simulationconstructionset.gui;

import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.RowConstraints;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFX3DGraph;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;

import javax.swing.*;
import java.awt.*;
import java.awt.print.PageFormat;
import java.awt.print.Printable;
import java.util.ArrayList;

public class GraphArrayPanel extends GridPane implements GraphIndicesHolder, YoGraphRemover, DataBufferChangeListener, Printable, ZoomGraphCommandExecutor
{
   private ArrayList<JavaFX3DGraph> javaFX3DGraphs;

   private StandardSimulationGUI standardSimulationGUI;
   private JFrame parentFrame;
   private DataBuffer dataBuffer;

   private int numColumns = 1;
   public final int MAX_GRAPHS = 24;
   public final int MAX_COLS = 4;

   private int leftPlotIndex;
   private int rightPlotIndex;

   private SelectedVariableHolder selectedVariableHolder;

   public GraphArrayPanel(SelectedVariableHolder holder, DataBuffer buffer, JFrame frame, StandardSimulationGUI standardSimulationGUI)
   {
      super();

      this.selectedVariableHolder = holder;

      this.standardSimulationGUI = standardSimulationGUI;
      this.parentFrame = frame;
      this.dataBuffer = buffer;

      leftPlotIndex = 0;
      rightPlotIndex = getMaxIndex();

      this.javaFX3DGraphs = new ArrayList<>();

      this.resetRows();
      this.resetColumns();

      this.setPrefSize(800, 400);
   }

   public ArrayList<YoGraph> getGraphsOnThisPanel()
   {
      ArrayList<YoGraph> graphs = new ArrayList<>();
      for (Node n : this.getChildren())
      {
         if (n instanceof YoGraph)
         {
            graphs.add((YoGraph) n);
         }
      }
      return graphs;
   }

   private void packGraphs()
   {
      int row = 0;
      int col = 0;

      for (Node n : this.getChildren())
      {
         GridPane.setConstraints(n, col, row);

         if (col + 1 >= numColumns)
         {
            col = 0;
            row++;
         }
         else
         {
            col++;
         }
      }
   }

   private void resetRows()
   {
      this.getRowConstraints().clear();

      int num = this.getRowConstraints().size();

      for (Node n : this.getChildren())
      {
         if (GridPane.getRowIndex(n) >= this.getRowConstraints().size())
         {
            num = GridPane.getRowIndex(n);
            this.getRowConstraints().add(new RowConstraints());
         }
      }

      for (RowConstraints rc : this.getRowConstraints())
      {
         rc.setPercentHeight(100.0d / (double) num);
      }
   }

   private void resetColumns()
   {
      this.getColumnConstraints().clear();
      for (int i = 0; i < numColumns; ++i)
      {
         ColumnConstraints newColumn = new ColumnConstraints();
         newColumn.setPercentWidth(100.0d / (double) numColumns);
         this.getColumnConstraints().add(newColumn);
      }
   }

   private void refreshGraphs()
   {
      this.resetColumns();
      this.resetRows();
      this.packGraphs();

      for (Node n : this.getChildren())
      {
         if (n instanceof YoGraph)
         {
            YoGraph g = (YoGraph) n;
            g.refresh();
         }
      }

      this.updateGraphs();
   }

   public void setNumColumns(int numColumns)
   {
      this.numColumns = numColumns;

      this.refreshGraphs();
   }

   public void addColumn()
   {
      if (numColumns >= this.MAX_COLS)
      {
         return;
      }

      numColumns++;

      this.refreshGraphs();
   }

   public void subColumn()
   {
      if (numColumns <= 1)
      {
         return;
      }

      this.numColumns--;

      this.refreshGraphs();
   }

   @Override public void dataBufferChanged()
   {
      this.zoomFullView();
   }

   public void setInteractionEnable(boolean enable)
   {
      for (int i = 0; i < this.getChildren().size(); i++)
      {
         Node n = this.getChildren().get(i);
         if (n instanceof YoGraph)
         {
            YoGraph yoGraph = (YoGraph) n;
            yoGraph.setInteractionEnable(enable);
         }
      }
   }

   private int oldIndex = 99;

   public void repaintGraphs()
   {
      int index = this.getIndex();

      if (index == oldIndex && index == doIndex)
      {
         return;
      }

      boolean repaintAll = false;

      if (((index < this.getLeftPlotIndex()) || (index > this.getRightPlotIndex())) || dataBuffer.hasBufferChanged())
      {
         dataBuffer.resetBufferChanged();
         this.recenter();
         repaintAll = true;
      }

      for (int i = 0; i < this.getChildren().size(); i++)
      {
         Node n = this.getChildren().get(i);
         if (n instanceof YoGraph)
         {
            YoGraph g = (YoGraph) n;
            if (g.getNumVars() > 0)
            {
               if (repaintAll)
               {
                  g.repaintAllGraph();
               }
               else if (oldIndex != index)
               {
                  g.repaintPartialGraph((oldIndex < index ? Math.max(0, oldIndex) : Math.max(0, index)),
                        (oldIndex > Math.max(0, index + 1) ? oldIndex : Math.max(0, index + 1)));
               }
            }
         }
      }

      oldIndex = index;
   }

   private boolean isPainting = false;

   public boolean isPaintingPanel()
   {
      return isPainting;
   }

   public void goToInPointNow()
   {
      dataBuffer.gotoInPoint();
   }

   public void goToOutPointNow()
   {
      dataBuffer.gotoOutPoint();
   }

   public boolean tick(int n)
   {
      boolean ret = dataBuffer.tick(n);
      this.repaintGraphs();
      return ret;
   }

   @Override public int getInPoint()
   {
      return dataBuffer.getInPoint();
   }

   @Override public int getOutPoint()
   {
      return dataBuffer.getOutPoint();
   }

   @Override public int getIndex()
   {
      return dataBuffer.getIndex();
   }

   @Override public boolean isIndexAtOutPoint()
   {
      return (getIndex() == getOutPoint());
   }

   @Override public int getMaxIndex()
   {
      return dataBuffer.getBufferSize() - 1;
   }

   @Override public int getLeftPlotIndex()
   {
      return this.leftPlotIndex;
   }

   @Override public int getRightPlotIndex()
   {
      return this.rightPlotIndex;
   }

   @Override public void setLeftPlotIndex(int idx)
   {
      this.leftPlotIndex = idx;
      repaintGraphs();
   }

   @Override public void setRightPlotIndex(int idx)
   {
      this.rightPlotIndex = idx;
      repaintGraphs();
   }

   public void zoomFullView()
   {
      rightPlotIndex = getMaxIndex();
      leftPlotIndex = 0;
      updateGraphs();
   }

   @Override public void zoomIn()
   {
      zoomIn(2);
   }

   public void zoomIn(int factor)
   {
      int index = this.getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;
      int newLength = oldLength / factor;

      if (newLength < 4)
         return;

      leftPlotIndex = index - newLength / 2; // index - (index-leftPlotIndex)/factor;
      rightPlotIndex = leftPlotIndex + newLength; // index + (rightPlotIndex - index)/factor;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;
         rightPlotIndex = leftPlotIndex + newLength;
         if (rightPlotIndex > getMaxIndex())
            rightPlotIndex = getMaxIndex();
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();
         leftPlotIndex = rightPlotIndex - newLength;
         if (leftPlotIndex < 0)
            leftPlotIndex = 0;
      }
   }

   @Override public void zoomOut()
   {
      zoomOut(2);
   }

   public void repaint()
   {
      repaintGraphs();
   }

   public void zoomOut(int factor)
   {
      int index = this.getIndex();

      int oldLength = rightPlotIndex - leftPlotIndex;
      int newLength = oldLength * factor;

      leftPlotIndex = index - newLength / 2; // index - (index-leftPlotIndex)*factor;
      rightPlotIndex = leftPlotIndex + newLength; // index + (rightPlotIndex - index)*factor;

      if (leftPlotIndex < 0)
      {
         leftPlotIndex = 0;
         rightPlotIndex = leftPlotIndex + newLength;
         if (rightPlotIndex > getMaxIndex())
            rightPlotIndex = getMaxIndex();
      }
      else if (rightPlotIndex > getMaxIndex())
      {
         rightPlotIndex = getMaxIndex();
         leftPlotIndex = rightPlotIndex - newLength;
         if (leftPlotIndex < 0)
            leftPlotIndex = 0;
      }
   }

   public void recenter()
   {
      zoomIn(1);
   }

   private int doTick = 0;
   private int doIndex = -1;

   @Override public void tickLater(int n)
   {
      if (dataBuffer.isKeyPointModeToggled())
      {
         if (n > 0)
         {
            setIndexLater(dataBuffer.getNextTime());
         }
         else
         {
            setIndexLater(dataBuffer.getPreviousTime());
         }
      }
      else
      {
         this.doTick = n;
      }

      this.repaintGraphs();
   }

   @Override public void setIndexLater(int idx)
   {
      this.doIndex = idx;
   }

   public boolean allowTickUpdatesNow()
   {
      boolean ret = false;

      if (this.doTick != 0)
      {
         dataBuffer.tick(doTick);

         this.repaintGraphs();
         ret = true;
         doTick = 0;
      }

      if (this.doIndex != -1)
      {
         dataBuffer.setIndex(this.doIndex);
         this.doIndex = -1;

         this.repaintGraphs();
         ret = true;
      }

      return ret;

   }

   public void setupGraph(String varname)
   {
      final DataBufferEntry entry = dataBuffer.getEntry(varname);

      if (entry != null)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override public void run()
            {
               YoGraph g = new YoGraph(getGraphArrayPanel(), getGraphArrayPanel(), selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
               g.addVariable(entry);
               addGraph(g);
            }

         });
      }
   }

   private GraphArrayPanel getGraphArrayPanel()
   {
      return this;
   }

   public void setupGraph(final String[] varnames)
   {
      if (varnames == null)
         return;

      EventDispatchThreadHelper.invokeAndWait(() ->
      {
         YoGraph g = new YoGraph(getGraphArrayPanel(), getGraphArrayPanel(), selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);

         for (int i = 0; i < varnames.length; i++)
         {
            DataBufferEntry entry = dataBuffer.getEntry(varnames[i]);

            if (entry != null)
               g.addVariable(entry);
         }

         addGraph(g);
      });
   }

   public void setupGraph(String[] varnames, GraphConfiguration config)
   {
      if (varnames == null)
         return;

      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      for (int i = 0; i < varnames.length; i++)
      {
         DataBufferEntry entry = dataBuffer.getEntry(varnames[i]);
         if (entry != null)
            g.addVariable(entry);
      }

      if (config != null)
         g.setGraphConfiguration(config);
      addGraph(g);
   }

   public void addSelectedVariableGraph()
   {
      YoVariable<?> variable = selectedVariableHolder.getSelectedVariable();
      DataBufferEntry entry = dataBuffer.getEntry(variable);
      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      g.addVariable(entry);
      addGraph(g);
   }

   public void addEmptyGraph()
   {
      YoGraph g = new YoGraph(this, this, selectedVariableHolder, dataBuffer, dataBuffer, parentFrame);
      addGraph(g);
   }

   public void addNew3dGraph()
   {
      JavaFX3DGraph javaFX3DGraph = new JavaFX3DGraph(this, selectedVariableHolder, dataBuffer, dataBuffer);
      javaFX3DGraphs.add(javaFX3DGraph);
      standardSimulationGUI.setupExtraPanels(new ExtraPanelConfiguration("3D Graph " + javaFX3DGraphs.size(), javaFX3DGraph.getPanel(), true));
      standardSimulationGUI.selectPanel("3D Graph " + javaFX3DGraphs.size());
   }

   public void removeEmptyGraphs()
   {
      YoGraph emptyGraph = null;

      // Get the last one that has zero elements that needs to be deleted

      for (int i = 0; i < this.getChildren().size(); i++)
      {
         Node n = this.getChildren().get(i);
         if (n instanceof YoGraph)
         {
            YoGraph graph = (YoGraph) n;
            if (graph.getNumVars() == 0)
            {
               emptyGraph = graph;
            }
         }
      }

      if (emptyGraph != null)
      {
         final YoGraph removeGraph = emptyGraph;
         Platform.runLater(() -> this.getChildren().remove(removeGraph));
         removeEmptyGraphs();
      }
      else
      {
         this.resetColumns();
         this.resetRows();
         this.packGraphs();
         this.updateGraphs();
      }
   }

   private synchronized int[] nextAvailableGraphLocation()
   {
      for (int row = 0; true; ++row)
      {
         thisCell:
         for (int col = 0; col < numColumns; ++col)
         {
            for (Node aGraphsOnThisPanel : this.getChildren())
            {
               if (getColumnIndex(aGraphsOnThisPanel) == col && getRowIndex(aGraphsOnThisPanel) == row)
               {
                  continue thisCell;
               }
            }
            return new int[] {row, col};
         }
      }
   }

   public void addGraph(YoGraph graph)
   {
      int numGraphs = this.getChildren().size();

      if (numGraphs >= this.MAX_GRAPHS)
      {
         return;
      }

      Platform.runLater(() ->
      {
         int[] useThis = nextAvailableGraphLocation();

         GridPane.setConstraints(graph, useThis[1], useThis[0]);

         graph.setStyle(
               "-fx-border-style: solid bevel;" + "-fx-border-width: 1px;" + "-fx-border-radius: 3px;" + "-fx-border-color: black;" + "-fx-margin: 5px");

            /*GridPane.setVgrow(graph, Priority.ALWAYS);
            GridPane.setHgrow(graph, Priority.ALWAYS);
            GridPane.setFillHeight(graph, true);
            GridPane.setFillWidth(graph, true);*/

         this.getChildren().add(graph);

         this.packGraphs();
         this.resetColumns();
         this.resetRows();
         this.updateGraphs();
      });
   }

   @Override public int print(Graphics graphics, PageFormat pageFormat, int pageNumber)
   {
      Graphics2D g2 = (Graphics2D) graphics;

      if (pageNumber == 0)
      {
         // First clear the graphics...
         // g2.setColor(Color.white);

         // g2.clearRect((int)pageFormat.getImageableX(),(int)pageFormat.getImageableY(),(int)pageFormat.getImageableWidth(),(int)pageFormat.getImageableHeight());
         // g2.setColor(Color.black);

         g2.translate(pageFormat.getImageableX(), pageFormat.getImageableY());

         // Scale:
         double pageWidth = pageFormat.getImageableWidth();
         double pageHeight = pageFormat.getImageableHeight();

         //YoGraph graph = this.graphsOnThisPanel.get(0);

         // double width = graph.getWidth();
         // double height = this.graphsOnThisPanel.size() * graph.getHeight() * 1.25;

         // double scaleX = pageWidth/width;
         // double scaleY = pageHeight/height;

         // double scaleFactor = Math.min(scaleX, scaleY);

         // g2.scale(scaleFactor, scaleFactor);

         for (int i = 0; i < this.getChildren().size(); i++)
         {
            Node n = this.getChildren().get(i);
            if (n instanceof YoGraph)
            {
               YoGraph g = (YoGraph) n;

               if (g.getEntriesOnThisGraph().size() > 0)
               {
                  // graph.paint(g2);
                  g.printGraph((int) pageWidth, (int) (pageHeight / 10.0));

                  // g2.translate(0.0, graph.getHeight()*1.25);
                  g2.translate(0.0, pageHeight / 8.0);
               }
            }
         }

         // this.paint(g2);

         // g2.drawRect(10,10,50,50);
         return Printable.PAGE_EXISTS;
      }

      return Printable.NO_SUCH_PAGE;
   }

   public void closeAndDispose()
   {
      parentFrame = null;
      dataBuffer = null;

      selectedVariableHolder = null;

      this.getChildren().clear();
   }

   @Override public void removeGraph(YoGraph graph)
   {
      Platform.runLater(() ->
      {
         this.getChildren().remove(graph);
         this.packGraphs();
         this.resetRows();
         this.resetColumns();
         this.updateGraphs();
      });
   }

   public void removeAllGraphs()
   {
      Platform.runLater(() ->
      {
         this.getChildren().clear();
         this.packGraphs();
         this.resetRows();
         this.resetColumns();
         this.updateGraphs();
      });
   }

   public JPanel createGraphButtonPanel()
   {
      JPanel graphButtonPanel = new JPanel();

      JButton newGraphButton = new JButton("New Graph");
      newGraphButton.setName("New Graph");
      newGraphButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addEmptyGraph();
         }
      });
      graphButtonPanel.add(newGraphButton);

      JButton removeEmptyGraphsButton = new JButton("Remove Empty");
      removeEmptyGraphsButton.setName("Remove Empty");
      removeEmptyGraphsButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            removeEmptyGraphs();
         }
      });
      graphButtonPanel.add(removeEmptyGraphsButton);

      JButton addColumnButton = new JButton("Add Column");
      addColumnButton.setName("Add Column");
      addColumnButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addColumn();
         }
      });
      graphButtonPanel.add(addColumnButton);

      JButton subColumnButton = new JButton("Sub Column");
      subColumnButton.setName("Sub Column");
      subColumnButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            subColumn();
         }
      });
      graphButtonPanel.add(subColumnButton);

      JButton new3DGraphButton = new JButton("New 3D Graph");
      new3DGraphButton.setName("New 3D Graph");
      new3DGraphButton.addActionListener(new java.awt.event.ActionListener()
      {
         @Override public void actionPerformed(java.awt.event.ActionEvent evt)
         {
            addNew3dGraph();
         }
      });
      graphButtonPanel.add(new3DGraphButton);

      return graphButtonPanel;
   }

   public String getXMLRepresentationOfClass()
   {
      String returnString = "<GraphGroup>\n";
      returnString += "\t<Cols>" + numColumns + "</Cols>\n";

      for (int j = 0; j < this.getChildren().size(); j++)
      {
         Node n = this.getChildren().get(j);
         if (n instanceof YoGraph)
         {
            YoGraph graph = (YoGraph) n;

            returnString += "\t<Graph>\n";
            returnString += "\t\t<Variables>";

            if (graph.getEntriesOnThisGraph().size() > 0)
            {
               returnString += graph.getEntriesOnThisGraph().get(0).getFullVariableNameWithNameSpace();

               for (int i = 1; i < graph.getEntriesOnThisGraph().size(); i++)
               {
                  returnString += "," + graph.getEntriesOnThisGraph().get(i).getFullVariableNameWithNameSpace();
               }
            }

            returnString += "</Variables>";
            returnString += "\n" + graph.getGraphConfiguration().getXMLStyleRepresentationOfClass();
            returnString += "\n\t</Graph>\n";
         }
      }

      returnString += "</GraphGroup>";

      return returnString;
   }

   @Override public ArrayList<Integer> getKeyPoints()
   {
      return dataBuffer.getKeyPoints();
   }

   private void updateGraphs()
   {
      this.repaintGraphs();
   }
}
