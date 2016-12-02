package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.DataProgressPanManager;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 6/14/16.
 */
public class DataProgressPanManagerPropertiesGUI implements TitledPanePropertiesGUI
{

   private DataProgressPanManager dataProgressPanManager;
   private TitledPane titledPane;

   public DataProgressPanManagerPropertiesGUI(DataProgressPanManager dataProgressPanManager)
   {
      this.dataProgressPanManager = dataProgressPanManager;

      final DataProgressPanManagerPropertiesController panController = new DataProgressPanManagerPropertiesController(this.dataProgressPanManager);

      try
      {
         final URL url = getClass().getResource("/xml/DataProgressPanManagerProperties.fxml");
         final FXMLLoader fxmlLoader = new FXMLLoader(url);
         fxmlLoader.setController(panController);
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
