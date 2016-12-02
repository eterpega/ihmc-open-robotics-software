package us.ihmc.javaFXToolkit.charts;

import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.Scene;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.control.Accordion;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.TitledPane;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.charts.dataModel.dataManagers.DynamicChart;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.*;
import us.ihmc.javaFXToolkit.charts.presentationModel.DataSeriesAppearanceManager;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.ChartGraphicsLayersContainer;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.SelectionRectangleGraphics;
import us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers.VerticalMarkerGraphics;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers.ChartInfoPropertiesGUI;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers.PanManagerPropertiesGUI;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers.ZoomManagerPropertiesGUI;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.dataSeries.DataSeriesAppearancePropertiesGUI;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers.SelectionRectanglePropertiesGUI;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.graphicsLayers.VerticalMarkerPropertiesGUI;

import java.net.URL;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 6/6/16.
 */
public class CustomizableChartController implements Initializable
{

   private static final int PROPERTIES_WINDOW_MAX_HEIGHT = 300;
   private static final int PROPERTIES_WINDOW_MAX_WIDTH = 300;
   private static final int DATASERIES_PROPERTIES_WINDOW_MAX_WIDTH = 300;
   private static final int DATASERIES_PROPERTIES_WINDOW_MAX_HEIGHT = 300;

   @FXML private ImageView settingsIcon;
   @FXML private ImageView dataSettingsIcon;
   @FXML private BorderPane borderPane;

   private boolean isPropertiesMenuVisible = false;
   private boolean isDataSeriesPropertiesGUIVisible = false;

   private final ChartGraphicsLayersContainer chartGraphicsLayersContainer;

   private final PanManager chartPanManager;
   private final ZoomManager chartZoomManager;
   private final ChartInformationManager chartInfoManager;
   private final SelectionRectangleGraphics selectionRectangle;
   private final ChartAxesTickManager chartAxesTickManager;

   private final VerticalMarkerGraphics verticalMarker;

   private final LineChart<Number, Number> chart;

   private Stage tempPropertiesStage;
   private Stage tempDataPropertiesStage;

   public CustomizableChartController(LineChart xyChart)
   {
      this.chart = xyChart;
      this.chartGraphicsLayersContainer = new ChartGraphicsLayersContainer(chart);

      this.chartPanManager = new PanManager(chart, -Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, Double.MAX_VALUE);
      this.chartZoomManager = new ZoomManager(chart);
      this.chartInfoManager = new ChartInformationManager(chart);

      this.selectionRectangle = new SelectionRectangleGraphics(chartGraphicsLayersContainer);
      this.selectionRectangle.selectionRectangleProperty().addListener(chartZoomManager.getSelectionRectangleChangeListener());

      this.verticalMarker = new VerticalMarkerGraphics(chartGraphicsLayersContainer);

      this.selectionRectangle.enableSelectionRectangleProperty().set(true);
      this.chartAxesTickManager = new ChartAxesTickManager((NumberAxis) chart.getXAxis(), (NumberAxis) chart.getYAxis());


      this.chartZoomManager.enableAllBehaviors(true);// TODO remove that when I have better way to initialize GUI configuration
   }

   public CustomizableChartController()
   {
      this(new LineChart(new NumberAxis(), new NumberAxis()));
   }

   public CustomizableChartController(NumberAxis xAxis, NumberAxis yAxis)
   {
      this(new LineChart(xAxis, yAxis));
   }

   public void closeChildrenWindows()
   {
      if (tempPropertiesStage != null)
      {
         tempPropertiesStage.close();
         tempPropertiesStage = null;
      }

      if (tempDataPropertiesStage != null)
      {
         tempDataPropertiesStage.close();
         tempDataPropertiesStage = null;
      }
   }

   @Override public void initialize(URL location, ResourceBundle resources)
   {
      assert settingsIcon != null : "fx:id=\"settingsIcon\" was not injected: check your FXML file 'CustomizableXYChart.fxml'.";
      assert dataSettingsIcon != null : "fx:id=\"dataSettingsIcon\" was not injected: check your FXML file 'CustomizableXYChart.fxml'.";
      assert borderPane != null : "fx:id=\"borderPane\" was not injected: check your FXML file 'CustomizableXYChart.fxml'.";

      borderPane.setCenter(chartGraphicsLayersContainer);

      this.settingsIcon.setOnMouseClicked(new EventHandler<MouseEvent>()
      {
         @Override public void handle(MouseEvent event)
         {
            if (!isPropertiesMenuVisible)
            {
               isPropertiesMenuVisible = true;

               tempPropertiesStage = new Stage();
               tempPropertiesStage.setMaxWidth(PROPERTIES_WINDOW_MAX_WIDTH);
               tempPropertiesStage.setMaxHeight(PROPERTIES_WINDOW_MAX_HEIGHT);
               Pane pane = createChartPropertiesGUI();
               Scene scene = new Scene(pane);
               tempPropertiesStage.setResizable(true);

               pane.widthProperty().addListener((observable, oldValue, newValue) -> {
                  tempPropertiesStage.setWidth(pane.getWidth());
               });

               pane.heightProperty().addListener((observable, oldValue, newValue) -> {
                  tempPropertiesStage.setHeight(pane.getHeight());
               });

               tempPropertiesStage.setScene(scene);
               tempPropertiesStage.show();

               tempPropertiesStage.setOnCloseRequest(event1 -> {
                  isPropertiesMenuVisible = false;
               });
            }
         }
      });

      this.dataSettingsIcon.setOnMouseClicked(event -> {

         if (!isDataSeriesPropertiesGUIVisible)
         {
            isDataSeriesPropertiesGUIVisible = true;

            tempDataPropertiesStage = new Stage();
            tempDataPropertiesStage.setMaxWidth(DATASERIES_PROPERTIES_WINDOW_MAX_WIDTH);
            tempDataPropertiesStage.setMaxHeight(DATASERIES_PROPERTIES_WINDOW_MAX_HEIGHT);
            DataSeriesAppearanceManager dataSeriesAppearanceManager = new DataSeriesAppearanceManager(chart);
            DataSeriesAppearancePropertiesGUI dataSeriesPropertiesGUI = new DataSeriesAppearancePropertiesGUI(dataSeriesAppearanceManager);
            tempDataPropertiesStage.setScene(new Scene(dataSeriesPropertiesGUI));
            tempDataPropertiesStage.show();

            tempDataPropertiesStage.setOnCloseRequest(event1 -> {
               isDataSeriesPropertiesGUIVisible = false;
            });
         }
      });

   }

   private Pane createChartPropertiesGUI()
   {
      AnchorPane anchorPane = new AnchorPane();
      anchorPane.setPrefSize(PROPERTIES_WINDOW_MAX_WIDTH, PROPERTIES_WINDOW_MAX_HEIGHT);
      Accordion accordion = new Accordion();
      ScrollPane scrollPane = new ScrollPane(accordion);
      anchorPane.getChildren().add(scrollPane);

      ChartInfoPropertiesGUI infoGUI = new ChartInfoPropertiesGUI(chartInfoManager);
      TitledPane infoTitledPane = infoGUI.getTitledPane();
      accordion.getPanes().add(infoTitledPane);
      // open this pane by default
      accordion.setExpandedPane(infoTitledPane);

      ZoomManagerPropertiesGUI zoomGUI = new ZoomManagerPropertiesGUI(chartZoomManager);
      TitledPane zoomTitledPane = zoomGUI.getTitledPane();
      accordion.getPanes().add(zoomTitledPane);

      SelectionRectanglePropertiesGUI selectionRectanglePropertiesGUI = new SelectionRectanglePropertiesGUI(selectionRectangle);
      accordion.getPanes().add(selectionRectanglePropertiesGUI.getTitledPane());

      PanManagerPropertiesGUI panGUI = new PanManagerPropertiesGUI(chartPanManager);
      accordion.getPanes().add(panGUI.getTitledPane());

      VerticalMarkerPropertiesGUI verticalMarkerPropertiesGUI = new VerticalMarkerPropertiesGUI(verticalMarker);
      accordion.getPanes().add(verticalMarkerPropertiesGUI.getTitledPane());

      AnchorPane.setBottomAnchor(scrollPane, 2.0);
      AnchorPane.setTopAnchor(scrollPane, 2.0);
      AnchorPane.setLeftAnchor(scrollPane, 2.0);
      AnchorPane.setRightAnchor(scrollPane, 2.0);

      return anchorPane;
   }

}