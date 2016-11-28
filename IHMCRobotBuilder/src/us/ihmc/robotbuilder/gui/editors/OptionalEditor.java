package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import us.ihmc.robotbuilder.gui.Creator;
import us.ihmc.robotbuilder.gui.Editor;

import java.util.Optional;
import java.util.function.Function;

import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class OptionalEditor<T> extends Editor<Optional<T>>
{
   private final BorderPane editorUi = new BorderPane();
   private final Button addButton = new Button("Add");
   private Editor<T> editor;
   private final Property<T> editingProperty = noCycle(new SimpleObjectProperty<>());
   private final Editor.Factory editorFactory;

   public OptionalEditor(Class<?> valueType, Property<Optional<T>> valueProperty, Editor.Factory editorFactory, Creator.Factory creatorFactory)
   {
      super(noCycle(valueProperty));
      this.editorFactory = editorFactory;

      ChangeListener<Optional<T>> changeListener = (observable, oldValue, newValue) ->
      {
         if (newValue.isPresent())
         {
            editorUi.setCenter(createEditor(valueType).getEditor());
         }
         else
         {
            editorUi.setCenter(addButton);
         }

      };

      valueProperty().addListener(changeListener);
      functional(valueProperty()).flatMapOptional(Function.identity()).consume(editingProperty::setValue);
      functional(editingProperty).map(Optional::ofNullable).consume(valueProperty()::setValue);

      Optional<Creator<T>> creatorOpt = creatorFactory.create(valueType);
      addButton.setDisable(!creatorOpt.isPresent());
      creatorOpt.ifPresent(creator -> addButton.setOnAction(event -> creator.create().onSuccess(valueProperty()::setValue)));
   }

   private Editor<T> createEditor(Class<?> valueType)
   {
      if (editor != null)
         return editor;

      //noinspection unchecked
      editor = (Editor<T>)editorFactory.create(valueType, editingProperty).orElse(new DummyEditor<>(editingProperty));
      return editor;
   }

   @Override public Node getEditor()
   {
      return editorUi;
   }
}
