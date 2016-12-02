package us.ihmc.javaFXToolkit.charts.presentationModel.propertiesGUI.dataSeries;

import javafx.fxml.FXMLLoader;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import us.ihmc.javaFXToolkit.charts.presentationModel.DataSeriesAppearanceManager;

import java.io.IOException;
import java.net.URL;

/**
 * Created by amoucheboeuf on 6/8/16.
 */
public class DataSeriesAppearancePropertiesGUI extends Pane
{

   private final DataSeriesAppearancePropertiesController controller;
   private AnchorPane anchorPane;

   public DataSeriesAppearancePropertiesGUI(DataSeriesAppearanceManager dataSeriesAppearanceManager)
   {
      controller = new DataSeriesAppearancePropertiesController(dataSeriesAppearanceManager);

      try
      {
         final URL url = getClass().getResource("/xml/DataSeriesProperties.fxml");
         final FXMLLoader fxmlLoader = new FXMLLoader(url);
         fxmlLoader.setController(controller);
         anchorPane = fxmlLoader.load();

      }
      catch (IOException ex)
      {
         System.err.println("Error loading: " + ex);
      }

      this.getChildren().add(anchorPane);

   }


}


