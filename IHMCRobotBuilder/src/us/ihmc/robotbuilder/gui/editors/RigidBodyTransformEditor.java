package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Node;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.immutableRobotDescription.graphics.TransformDescription;

import javax.vecmath.Matrix4f;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class RigidBodyTransformEditor extends Editor<RigidBodyTransform>
{
   private final TransformEditor transformEditor;
   private final Property<TransformDescription> transformDescriptionProperty = noCycle(new SimpleObjectProperty<>());

   public RigidBodyTransformEditor(Property<RigidBodyTransform> valueProperty)
   {
      super(noCycle(valueProperty));
      functional(transformDescriptionProperty).map(this::transformDescriptionToRigidBodyTransform).consume(valueProperty()::setValue);
      functional(valueProperty()).map(this::rigidBodyTransformToTransformDescription).consume(transformDescriptionProperty::setValue);
      transformEditor = new TransformEditor(transformDescriptionProperty);
   }

   private RigidBodyTransform transformDescriptionToRigidBodyTransform(TransformDescription transformDescription)
   {
      return new RigidBodyTransform(transformDescription.getMatrix());
   }

   private TransformDescription rigidBodyTransformToTransformDescription(RigidBodyTransform rigidBodyTransform)
   {
      Matrix4f matrix = new Matrix4f();
      rigidBodyTransform.get(matrix);
      return new TransformDescription(matrix);
   }

   @Override public Node getEditor()
   {
      return transformEditor.getEditor();
   }
}
