package us.ihmc.robotbuilder.gui.editors;

import javafx.event.ActionEvent;
import javafx.scene.Node;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.Dialog;
import javafx.stage.StageStyle;

/**
 * A dialog based UI for {@link us.ihmc.robotbuilder.gui.Creator}.
 */
public class DialogCreatorUI extends AbstractCreatorUI
{
   private final Dialog<Object> dialog = new Dialog<>();

   public DialogCreatorUI(Node editor)
   {
      dialog.initStyle(StageStyle.UTILITY);
      dialog.getDialogPane().setContent(editor);
      dialog.setResultConverter(dialogButton -> {
         if (dialogButton == ButtonType.CANCEL)
         {
            fireCancel();
         }
         return null;
      });

      dialog.getDialogPane().getButtonTypes().addAll(ButtonType.OK, ButtonType.CANCEL);

      final Button okButton = (Button) dialog.getDialogPane().lookupButton(ButtonType.OK);
      okButton.addEventFilter(ActionEvent.ACTION, ae -> fireConfirm().ifPresent(error -> {
         ae.consume();
         new Alert(AlertType.ERROR, error.getMessage(), ButtonType.OK).show();
      }));

      dialog.show();
   }
}
