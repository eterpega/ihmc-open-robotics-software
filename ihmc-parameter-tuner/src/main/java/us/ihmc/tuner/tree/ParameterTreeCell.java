package us.ihmc.tuner.tree;

import javafx.scene.control.TreeCell;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.tuner.util.ParameterNameUtil;

/**
 * A single cell in the parameter tree. Supports drag-and-drop if the cell contains a parameter.
 */
public class ParameterTreeCell extends TreeCell<ParameterTreeValue>
{
   private ParameterTreeValue treeValue;

   public ParameterTreeCell()
   {
      setOnDragDetected(event ->
      {
         // Only support dragging if this is a parameter. It's possible this is a package name, in which case it shouldn't be draggable.
         Parameter parameter = getAssociatedParameter();
         if (parameter != null)
         {
            Dragboard dragboard = startDragAndDrop(TransferMode.LINK);

            ClipboardContent content = new ClipboardContent();
            content.putString(treeValue.getDraggableParameterPath());
            dragboard.setContent(content);

            event.consume();
         }
      });
   }

   @Override
   protected void updateItem(ParameterTreeValue item, boolean empty)
   {
      super.updateItem(item, empty);

      if (empty || item == null)
      {
         setText(null);
      }
      else
      {
         this.treeValue = item;
         setText(item.toString());
      }
   }

   private Parameter getAssociatedParameter()
   {
      // Make sure there is an associated parameter (i.e. this isn't a package header, array header, etc.).
      if (treeValue != null && treeValue.getDraggableParameterPath() != null)
      {
         String parameterPath = ParameterNameUtil.getParameterNameFromDraggablePath(treeValue.getDraggableParameterPath());
         if (parameterPath != null)
         {
            return ParameterRegistry.getInstance().getParameter(parameterPath);
         }
      }

      return null;
   }
}
