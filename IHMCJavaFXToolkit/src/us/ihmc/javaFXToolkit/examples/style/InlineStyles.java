package us.ihmc.javaFXToolkit.examples.style;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

/**
 * Created by amoucheboeuf on 7/27/16.
 */
public class InlineStyles extends Application
{

   public static void main(String[] args)
   {
      Application.launch(args);
   }

   @Override
   public void start(Stage stage)
   {
      Button yesButton = new Button("Yes");
      Button noButton = new Button("No");
      Button cancelButton = new Button("Cancel");

      // Add a inline style to the Yes button

      yesButton.setStyle("-fx-text-fill: red; -fx-font-weight: bold;");

      VBox root = new VBox();
      root.setPadding(new Insets(10, 10, 10, 10));
      root.getChildren().addAll(yesButton, noButton, cancelButton);

      // Add border to VBoxes using an inline style
      root.setStyle("-fx-border-width: 4.0; -fx-border-color: blue;");

      Scene scene = new Scene(root);
      stage.setScene(scene);
      stage.setTitle("Using Inline Styles");
      stage.show();
   }

}
