package us.ihmc.robotbuilder.gui.editors;

import javafx.scene.Node;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;
import org.controlsfx.property.editor.PropertyEditor;

import javax.vecmath.Vector3d;

/**
 *
 */
public class Vector3DEditor implements PropertyEditor<Vector3d>
{
   private final EditorComponent editorComponent = new EditorComponent();

   @Override public Node getEditor()
   {
      return editorComponent;
   }

   @Override public Vector3d getValue()
   {
      return editorComponent.getValue();
   }

   @Override public void setValue(Vector3d value)
   {
      editorComponent.setValue(value);
   }

   private static class EditorComponent extends GridPane
   {
      private final DecimalTextField[] textFields = new DecimalTextField[3];

      EditorComponent()
      {
         for (int i = 0; i < textFields.length; i++)
         {
            textFields[i] = new DecimalTextField();
            GridPane.setColumnIndex(textFields[i], i);
         }
         getChildren().addAll(textFields);
      }

      void setValue(Vector3d value)
      {
         textFields[0].setText(Double.toString(value.x));
         textFields[1].setText(Double.toString(value.y));
         textFields[2].setText(Double.toString(value.z));
      }

      Vector3d getValue()
      {
         Vector3d result = new Vector3d();
         result.x = textFields[0].getValue();
         result.y = textFields[1].getValue();
         result.z = textFields[2].getValue();
         return result;
      }
   }
}
