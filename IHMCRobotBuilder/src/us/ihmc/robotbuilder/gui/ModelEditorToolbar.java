package us.ihmc.robotbuilder.gui;

import javafx.geometry.Point2D;
import javafx.scene.control.Button;
import javafx.scene.control.ToolBar;
import javafx.scene.image.Image;
import javafx.scene.layout.*;

import java.util.Arrays;
import java.util.stream.Collectors;

public class ModelEditorToolbar extends ToolBar
{
   @SuppressWarnings("unchecked") public ModelEditorToolbar()
   {
      getItems().addAll(Arrays.stream(JointDescriptionLabels.getAllLabeledJoints().keySet().toArray()).map(action ->
      {
         Button button = new Button();

         button.getStyleClass().add("modelEditorButton");

         button.setBackground(new Background(
               new BackgroundImage(new Image(getClass().getResource("/icons/" + action + ".png").toExternalForm()), BackgroundRepeat.NO_REPEAT,
                     BackgroundRepeat.NO_REPEAT, BackgroundPosition.CENTER, BackgroundSize.DEFAULT)));

         button.setOnMousePressed(
               me -> JointDescriptionLabels.getJointDescriptionFromLabel((String) action).ifPresent(JointDropHolder.getInstance()::setJoint));

         button.setOnMouseReleased(me ->
         {
            System.out.println("released");
            if (Preview3D.getInstance().localToScene(Preview3D.getInstance().getBoundsInLocal()).contains(new Point2D(me.getSceneX(), me.getSceneY())))
            {
               System.out.println("P3D contains point");
               // TODO: create mouse event with source as graphics joint; might use Robot or similar to click
               //Preview3D.getInstance().handleGraphicsClick(me);
               if (Preview3D.getInstance().jointTreeProperty() != null)
               {
                  System.out.println("joint tree not null");
                  Preview3D.getInstance().jointTreeProperty().getValue().ifPresent(treeFocus ->
                  {
                     System.out.println("tree present");
                     JointDropHolder.getInstance().getJoint().ifPresent(joint ->
                     {
                        try
                        {
                           System.out.println("trying to create joint at tree point");
                           //treeFocus.addLeftSibling(new Tree<JointDescription>(joint.getConstructor().newInstance("")));
                        }
                        catch (Exception e)
                        {
                           e.printStackTrace();
                        }
                     });
                  });
               }
            }
         });

         return button;
      }).collect(Collectors.toList()));
   }
}
