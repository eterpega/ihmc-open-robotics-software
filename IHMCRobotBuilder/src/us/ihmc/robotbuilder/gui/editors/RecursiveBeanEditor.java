package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.PropertyEditor;

import java.util.Stack;

/**
 *
 */
public class RecursiveBeanEditor<T> implements PropertyEditor<T>
{
   private final RecursiveEditorFactory FACTORY = new RecursiveEditorFactory();

   private final BorderPane container = new BorderPane();
   private final Stack<ImmutableBeanEditor<?>> editorStack = new Stack<>();
   private final Property<T> valueProperty;

   public RecursiveBeanEditor(T objectToEdit)
   {
      ImmutableBeanEditor<T> rootEditor = new ImmutableBeanEditor<>(objectToEdit, FACTORY, false);
      editorStack.push(rootEditor);
      valueProperty = rootEditor.valueProperty();
      updateEditorState();
   }

   private void updateEditorState()
   {
      if (editorStack.size() > 0)
         container.setCenter(editorStack.peek().getEditor());
      else
         container.setCenter(null);
   }

   @Override public Node getEditor()
   {
      return container;
   }

   @Override public T getValue()
   {
      return valueProperty.getValue();
   }

   public Property<T> valueProperty()
   {
      return valueProperty;
   }

   @Override public void setValue(T value)
   {
      // Traverse back to the root
      while (editorStack.size() > 1)
         editorStack.pop();

      valueProperty.setValue(value);
   }

   private class RecursiveEditorFactory extends EditorFactory
   {
      @Override public PropertyEditor<?> call(Item item)
      {
         PropertyEditor<?> editor = super.call(item);
         if (editor != null)
            return editor;

         if (item.isEditable())
            return new InnerBeanEditor(item);

         return null;
      }
   }

   private class InnerBeanEditor extends AbstractPropertyEditor<Object, FlowPane>
   {
      private ImmutableBeanEditor<Object> editor;

      InnerBeanEditor(Item property)
      {
         super(property, new FlowPane());
         Button edit = new Button("Edit...");
         edit.setOnAction(e ->
                                {
                                    editorStack.push(editor);
                                   updateEditorState();
                                });
         getEditor().getChildren().add(edit);
      }

      @Override protected ObservableValue<Object> getObservableValue()
      {
         if (editor == null)
            editor = new ImmutableBeanEditor<>(getProperty().getValue(), FACTORY, false);
         return editor.valueProperty();
      }

      @Override public void setValue(Object value)
      {
         editor.setValue(value);
      }
   }
}
