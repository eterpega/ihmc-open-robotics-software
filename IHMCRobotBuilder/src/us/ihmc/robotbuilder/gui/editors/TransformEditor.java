package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.InvalidationListener;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;

/**
 * Editor for {@link TransformDescription}.
 */
public class TransformEditor extends AbstractPropertyEditor<TransformDescription, GridPane>
{
   private Property<TransformDescription> value;

   private final Vector3DEditor rotationEditor, scaleEditor, translationEditor;
   private boolean editing = false;

   public TransformEditor(Item property)
   {
      super(property, new GridPane(), false);
      rotationEditor = new Vector3DEditor(new VectorItem(this::setRotation, this::getRotation));
      scaleEditor = new Vector3DEditor(new VectorItem(this::setScale, this::getScale));
      translationEditor = new Vector3DEditor(new VectorItem(this::setTranslation, this::getTranslation));

      getEditor().setHgap(15);
      getEditor().add(new Label("Rotation"), 0, 0);
      getEditor().add(new Label("Scale"), 0, 1);
      getEditor().add(new Label("Translation"), 0, 2);
      getEditor().add(rotationEditor.getEditor(), 1, 0);
      getEditor().add(scaleEditor.getEditor(), 1, 1);
      getEditor().add(translationEditor.getEditor(), 1, 2);

      InvalidationListener onChange = (unused) ->
      {
         if (editing)
            return;
         rotationEditor.setValue(getRotation());
         scaleEditor.setValue(getScale());
         translationEditor.setValue(getTranslation());
      };
      value.addListener(onChange);
      onChange.invalidated(null);
   }

   private void setRotation(Vector3d eulerAnglesInDegrees)
   {
      Vector3f eulerAnglesInRadians = new Vector3f((float)Math.toRadians(eulerAnglesInDegrees.x),
                                                   (float)Math.toRadians(eulerAnglesInDegrees.y),
                                                   (float)Math.toRadians(eulerAnglesInDegrees.z));
      value.setValue(value.getValue().withRotation(eulerAnglesInRadians));
   }

   private Vector3d getRotation()
   {
      Vector3f rotationRadians = value.getValue().getRotationEulerAngles();
      return new Vector3d(Math.toDegrees(rotationRadians.x), Math.toDegrees(rotationRadians.y), Math.toDegrees(rotationRadians.z));
   }

   private void setScale(Vector3d scale)
   {
      value.setValue(value.getValue().withScale(new Vector3f(scale)));
   }

   private Vector3d getScale()
   {
      return new Vector3d(value.getValue().getScale());
   }

   private void setTranslation(Vector3d translation)
   {
      value.setValue(value.getValue().withTranslation(new Vector3f(translation)));
   }

   private Vector3d getTranslation()
   {
      return new Vector3d(getValue().getTranslation());
   }

   @Override protected ObservableValue<TransformDescription> getObservableValue()
   {
      if (value == null)
         value = new SimpleObjectProperty<>(TransformDescription.IDENTITY);
      return value;
   }

   @Override public void setValue(TransformDescription value)
   {
      this.value.setValue(value);
   }

   private class VectorItem implements Item
   {
      final Consumer<Vector3d> valueSetter;
      final Supplier<Vector3d> valueGetter;

      private VectorItem(Consumer<Vector3d> valueSetter, Supplier<Vector3d> valueGetter)
      {
         this.valueSetter = valueSetter;
         this.valueGetter = valueGetter;
      }

      @Override public Class<?> getType()
      {
         return Vector3d.class;
      }

      @Override public String getCategory()
      {
         return "Basic";
      }

      @Override public String getName()
      {
         return "Vector Item";
      }

      @Override public String getDescription()
      {
         return null;
      }

      @Override public Object getValue()
      {
         return valueGetter.get();
      }

      @Override public void setValue(Object val)
      {
         editing = true;
         if (val instanceof Vector3d)
           valueSetter.accept((Vector3d)val);
         editing = false;
      }

      @Override public Optional<ObservableValue<?>> getObservableValue()
      {
         return Optional.of(functional(TransformEditor.this.getObservableValue()).map(x -> valueGetter.get()));
      }
   }
}
