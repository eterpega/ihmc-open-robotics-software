package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 * Editor for {@link TransformDescription}.
 */
public class TransformEditor extends Editor<TransformDescription>
{
   private final GridPane container = new GridPane();

   public TransformEditor(Property<TransformDescription> property)
   {
      super(noCycle(property));
      Property<Vector3d> rotationProperty = new SimpleObjectProperty<>();
      functional(rotationProperty).consume(this::setRotation);
      Property<Vector3d> scaleProperty = new SimpleObjectProperty<>();
      functional(scaleProperty).consume(this::setScale);
      Property<Vector3d> translationProperty = new SimpleObjectProperty<>();
      functional(translationProperty).consume(this::setTranslation);

      valueProperty().addListener((observable, oldValue, newValue) ->
                                  {
                                     rotationProperty.setValue(getRotation());
                                     scaleProperty.setValue(getScale());
                                     translationProperty.setValue(getTranslation());
                                  });

      Vector3DEditor rotationEditor = new Vector3DEditor(rotationProperty);
      Vector3DEditor scaleEditor = new Vector3DEditor(scaleProperty);
      Vector3DEditor translationEditor = new Vector3DEditor(translationProperty);

      container.setHgap(15);
      container.add(new Label("Rotation"), 0, 0);
      container.add(new Label("Scale"), 0, 1);
      container.add(new Label("Translation"), 0, 2);
      container.add(rotationEditor.getEditor(), 1, 0);
      container.add(scaleEditor.getEditor(), 1, 1);
      container.add(translationEditor.getEditor(), 1, 2);
   }

   @Override public Node getEditor()
   {
      return container;
   }

   private void setRotation(Vector3d eulerAnglesInDegrees)
   {
      Vector3f eulerAnglesInRadians = new Vector3f((float)Math.toRadians(eulerAnglesInDegrees.x),
                                                   (float)Math.toRadians(eulerAnglesInDegrees.y),
                                                   (float)Math.toRadians(eulerAnglesInDegrees.z));
      valueProperty().setValue(valueProperty().getValue().withRotation(eulerAnglesInRadians));
   }

   private Vector3d getRotation()
   {
      Vector3f rotationRadians = valueProperty().getValue().getRotationEulerAngles();
      return new Vector3d(Math.toDegrees(rotationRadians.x), Math.toDegrees(rotationRadians.y), Math.toDegrees(rotationRadians.z));
   }

   private void setScale(Vector3d scale)
   {
      valueProperty().setValue(valueProperty().getValue().withScale(new Vector3f(scale)));
   }

   private Vector3d getScale()
   {
      return new Vector3d(valueProperty().getValue().getScale());
   }

   private void setTranslation(Vector3d translation)
   {
      valueProperty().setValue(valueProperty().getValue().withTranslation(new Vector3f(translation)));
   }

   private Vector3d getTranslation()
   {
      return new Vector3d(valueProperty().getValue().getTranslation());
   }
}
