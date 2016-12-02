package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.CheckBox;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.ChartInformationManager;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class ChartInfoPropertiesController implements Initializable
{
   private List<CheckBox> checkBoxList = new ArrayList<>();

   @FXML private CheckBox showTitle;

   @FXML private CheckBox showLegend;

   @FXML private CheckBox showXAxis;

   @FXML private CheckBox showYAxis;

   @FXML private CheckBox showXLabel;

   @FXML private CheckBox showYLabel;

   private final ChartInformationManager chartInformationManager;

   public ChartInfoPropertiesController(ChartInformationManager chartInformationManager)
   {
      this.chartInformationManager = chartInformationManager;
   }


   @Override public void initialize(URL location, ResourceBundle resources)
   {
      assert showTitle != null : "fx:id=\"showTitle\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";
      assert showLegend != null : "fx:id=\"showTitle\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";
      assert showXAxis != null : "fx:id=\"showXAxis\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";
      assert showYAxis != null : "fx:id=\"showYAxis\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";
      assert showXLabel != null : "fx:id=\"showXLabel\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";
      assert showYLabel != null : "fx:id=\"showYLabel\" was not injected: check your FXML file 'ChartInfoProperties.fxml'.";

      checkBoxList.add(showTitle);
      checkBoxList.add(showLegend);
      checkBoxList.add(showXAxis);
      checkBoxList.add(showYAxis);
      checkBoxList.add(showXLabel);
      checkBoxList.add(showYLabel);


      showXLabel.disableProperty().bind(showXAxis.selectedProperty().not());
      showYLabel.disableProperty().bind(showYAxis.selectedProperty().not());

      chartInformationManager.xAxisVisibleProperty().bind(showXAxis.selectedProperty());
      chartInformationManager.yAxisVisibleProperty().bind(showYAxis.selectedProperty());

      chartInformationManager.xAxisLabelVisibleProperty().bind(showXLabel.selectedProperty());
      chartInformationManager.yAxisLabelVisibleProperty().bind(showYLabel.selectedProperty());

      chartInformationManager.titleVisibleProperty().bind(showTitle.selectedProperty());
      chartInformationManager.legendVisibleProperty().bind(showLegend.selectedProperty());


      setSelectedOnAllCheckBoxes(true);
   }

   public void setSelectedOnAllCheckBoxes(boolean selected)
   {
      for (CheckBox checkbox: checkBoxList)
      {
         checkbox.setSelected(selected);
      }
   }

}
