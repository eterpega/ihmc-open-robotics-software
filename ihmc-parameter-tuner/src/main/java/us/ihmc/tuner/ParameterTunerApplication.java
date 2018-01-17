package us.ihmc.tuner;

import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;

public class ParameterTunerApplication
{
   private static final String FXML_PATH = "param_tuner.fxml";

   private final PacketCommunicator packetCommunicator;

   public ParameterTunerApplication(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
   }

   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader mainLoader = new FXMLLoader(getClass().getResource(FXML_PATH));
      Scene mainScene = new Scene(mainLoader.<Pane>load());

      ParameterTunerController mainController = mainLoader.getController();
      mainController.attachParameterCommunicator(packetCommunicator);

      primaryStage.setScene(mainScene);
      primaryStage.show();
   }
}
