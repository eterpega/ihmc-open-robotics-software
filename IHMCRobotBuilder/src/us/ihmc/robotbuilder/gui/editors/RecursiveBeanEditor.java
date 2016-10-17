package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TreeItem;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import org.controlsfx.control.BreadCrumbBar;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.editor.AbstractPropertyEditor;
import org.controlsfx.property.editor.PropertyEditor;
import us.ihmc.robotics.immutableRobotDescription.NamedObject;

import java.util.Objects;
import java.util.Optional;
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
   private final BreadCrumbBar<BreadCrumbItem> breadCrumbBar = new BreadCrumbBar<>();

   public RecursiveBeanEditor(T objectToEdit)
   {
      ImmutableBeanEditor<T> rootEditor = new ImmutableBeanEditor<>(new RootProperty(objectToEdit), FACTORY, false);
      editorStack.push(rootEditor);
      valueProperty = rootEditor.valueProperty();
      container.setTop(breadCrumbBar);
      updateEditorState();

      breadCrumbBar.setOnCrumbAction(event -> revertTo(event.getSelectedCrumb().getValue().index));
   }

   private void updateEditorState()
   {
      if (editorStack.size() > 0)
         container.setCenter(editorStack.peek().getEditor());
      else
         container.setCenter(null);

      updateBreadcrumbs();
   }

   private void updateBreadcrumbs()
   {
      TreeItem<BreadCrumbItem> item = null;
      for (int i = 0; i < editorStack.size(); i++)
      {
         TreeItem<BreadCrumbItem> lastItem = item;
         item = new TreeItem<>(new BreadCrumbItem(i, editorStack.get(i).getProperty()));
         if (lastItem != null)
            lastItem.getChildren().add(item);
      }
      breadCrumbBar.setSelectedCrumb(item);
   }

   private void revertTo(int index)
   {
      if (index < 0)
         return;

      while (editorStack.size() > index + 1)
         editorStack.pop();
      updateEditorState();
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
            editor = new ImmutableBeanEditor<>(getProperty(), FACTORY, false);
         return editor.valueProperty();
      }

      @Override public void setValue(Object value)
      {
         editor.setValue(value);
      }
   }

   private static class BreadCrumbItem
   {
      private final int index;
      private final Item editedItem;

      private BreadCrumbItem(int index, Item editedItem)
      {
         this.index = index;
         this.editedItem = editedItem;
      }

      @Override public String toString()
      {
         return editedItem.getName();
      }
   }

   private class RootProperty implements Item
   {
      final T objectToEdit;

      private RootProperty(T objectToEdit)
      {
         this.objectToEdit = objectToEdit;
      }

      @Override public Class<?> getType()
      {
         return objectToEdit.getClass();
      }

      @Override public String getCategory()
      {
         return "";
      }

      @Override public String getName()
      {
         return objectToEdit instanceof NamedObject ? ((NamedObject)objectToEdit).getName() : Objects.toString(objectToEdit);
      }

      @Override public String getDescription()
      {
         return getName();
      }

      @Override public Object getValue()
      {
         return objectToEdit;
      }

      @Override public void setValue(Object value)
      {

      }

      @Override public Optional<ObservableValue<?>> getObservableValue()
      {
         return Optional.empty();
      }
   }
}
