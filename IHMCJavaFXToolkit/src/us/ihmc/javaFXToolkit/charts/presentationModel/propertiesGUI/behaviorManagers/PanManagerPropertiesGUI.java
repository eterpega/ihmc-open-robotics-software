package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.behaviorManagers;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TitledPane;
import us.ihmc.javaFXToolkit.charts.presentationModel.behaviorManagers.PanManager;
import us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.TitledPanePropertiesGUI;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 5/26/16.
 */
public class PanManagerPropertiesGUI implements TitledPanePropertiesGUI
{

   private PanManager chartPanManager;
   private TitledPane titledPane;

   public PanManagerPropertiesGUI(PanManager chartPanManager)
   {
      this.chartPanManager = chartPanManager;

      final PanManagerPropertiesController panController = new PanManagerPropertiesController(this.chartPanManager);

      try
      {
         final URL url = getClass().getResource("/xml/PanManagerProperties.fxml");
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


