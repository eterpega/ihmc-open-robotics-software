package us.ihmc.robotbuilder.gui.editors2;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.TreeItem;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import javaslang.Tuple;
import javaslang.Tuple2;
import org.controlsfx.control.BreadCrumbBar;
import org.controlsfx.property.editor.PropertyEditor;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotics.immutableRobotDescription.NamedObject;

import java.util.Objects;
import java.util.Optional;
import java.util.Stack;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;

/**
 *
 */
public class RecursiveBeanEditor<T> extends Editor<T>
{

   private final BorderPane container = new BorderPane();
   private final Stack<Tuple2<String, ImmutableEditor<?>>> editorStack = new Stack<>();
   private final BreadCrumbBar<BreadCrumbItem> breadCrumbBar = new BreadCrumbBar<>();

   public RecursiveBeanEditor(Property<T> valueProperty)
   {
      super(valueProperty);
      functional(valueProperty)
            .avoidCycles()
            .consume(objectToEdit -> {
               editorStack.clear();
               ImmutableEditor<T> rootEditor = new ImmutableEditor<>(valueProperty, new RecursiveEditorFactory());
               editorStack.push(Tuple.of(objectName(objectToEdit), rootEditor));
               container.setTop(breadCrumbBar);
               updateEditorState();
            });


      breadCrumbBar.setOnCrumbAction(event -> revertTo(event.getSelectedCrumb().getValue().index));
   }

   private static String objectName(Object object)
   {
      if (object instanceof NamedObject)
         return ((NamedObject) object).getName();
      return Objects.toString(object);
   }

   private void updateEditorState()
   {
      if (editorStack.size() > 0)
         container.setCenter(editorStack.peek()._2.getEditor());
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
         item = new TreeItem<>(new BreadCrumbItem(i, editorStack.get(i)._1));
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

   private class RecursiveEditorFactory extends PropertyEditorFactory
   {
      @Override public Optional<Editor<?>> create(Class<?> clazz, Property<?> value, String name)
      {
         return Optional.of(super.create(clazz, value, name).orElse(new InnerBeanEditor<>(value, this, name)));
      }
   }

   private class InnerBeanEditor<U> extends Editor<U>
   {
      private final ImmutableEditor<U> innerEditor;
      private final FlowPane container = new FlowPane();

      InnerBeanEditor(Property<U> property, Factory factory, String name)
      {
         super(property);
         innerEditor = new ImmutableEditor<>(property, factory);
         Button edit = new Button("Edit...");
         edit.setOnAction(e ->
                          {
                             editorStack.push(Tuple.of(name, innerEditor));
                             updateEditorState();
                          });
         container.getChildren().add(edit);
      }

      @Override public Node getEditor()
      {
         return container;
      }
   }

   private static class BreadCrumbItem
   {
      private final int index;
      private final String name;

      private BreadCrumbItem(int index, String name)
      {
         this.index = index;
         this.name = name;
      }

      @Override public String toString()
      {
         return name;
      }
   }
}
