package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.ChartInformationManager;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 5/31/16.
 */


public class ChartInfoPropertiesGUI implements TitledPanePropertiesGUI
{

   private ChartInformationManager chartInformationManager;
   private TitledPane titledPane;

   public ChartInfoPropertiesGUI(ChartInformationManager chartInformationManager)
   {
      this.chartInformationManager = chartInformationManager;

      final ChartInfoPropertiesController controller = new ChartInfoPropertiesController(this.chartInformationManager);

      try
      {
         final URL url = getClass().getResource("/xml/ChartInfoManagerProperties.fxml");
         final FXMLLoader fxmlLoader = new FXMLLoader(url);
         fxmlLoader.setController(controller);
         titledPane = (TitledPane) fxmlLoader.load();

      }
      catch (IOException ex)
      {
         System.err.println("Error loading: " + ex);
      }
   }

   public TitledPane getTitledPane()
   {
      return titledPane;
   }

}
