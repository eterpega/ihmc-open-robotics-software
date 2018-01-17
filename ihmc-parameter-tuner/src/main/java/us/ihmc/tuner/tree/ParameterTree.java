package us.ihmc.tuner.tree;

import java.io.IOException;
import java.util.List;

import javafx.fxml.FXMLLoader;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import us.ihmc.robotics.dataStructures.parameter.ArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tuner.util.ParameterNameUtil;

/**
 * A tree structure containing parameter package names, array parameter headers, and parameters.
 */
public class ParameterTree extends TreeView<ParameterTreeValue>
{
   private static final String FXML_PATH = "param_tree.fxml";

   // Empty ctor for FXML construction.
   public ParameterTree() throws IOException
   {
      this(null);
   }

   public ParameterTree(ParameterTreeValue root) throws IOException
   {
      super(new TreeItem<>(root));

      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      loader.load();

      setCellFactory(param -> new ParameterTreeCell());
   }

   public void setParameters(List<Parameter> parameters)
   {
      // Create a root element to add children, but don't show it. This is just because there needs to be a single root.
      ParameterTreeValue rootValue = new PackageHeaderParameterTreeValue("");
      TreeItem<ParameterTreeValue> root = new TreeItem<>(rootValue);
      root.setExpanded(true);
      super.setShowRoot(false);

      for (Parameter parameter : parameters)
      {
         // Check if this parameter's package is already in the view.
         String path = parameter.getPath();
         String packageName = ParameterNameUtil.getPackageNameFromQualifiedName(path);
         TreeItem<ParameterTreeValue> packageItem = findChild(root, packageName);
         if (packageItem == null)
         {
            // If not, add the package.
            packageItem = new TreeItem<>(new PackageHeaderParameterTreeValue(packageName));
            root.getChildren().add(packageItem);
         }

         // Array parameters need a header then a row for each element
         if (parameter instanceof ArrayParameter)
         {
            ArrayParameter arrayParameter = (ArrayParameter) parameter;

            TreeItem<ParameterTreeValue> arrayItem = new TreeItem<>(new ArrayHeaderParameterTreeValue(parameter));
            packageItem.getChildren().add(arrayItem);

            for (int i = 0; i < arrayParameter.length(); i++)
            {
               // Add the parameter to the package in the view.
               TreeItem<ParameterTreeValue> elementItem = new TreeItem<>(new ArrayElementParameterTreeValue(parameter, i));
               arrayItem.getChildren().add(elementItem);
            }
         }
         // Single parameter just need one element
         else
         {
            SingleParameterTreeValue item = new SingleParameterTreeValue(parameter);
            packageItem.getChildren().add(new TreeItem<>(item));
         }
      }

      super.setRoot(root);
   }

   private TreeItem<ParameterTreeValue> findChild(TreeItem<ParameterTreeValue> parent, String text)
   {
      for (TreeItem<ParameterTreeValue> item : parent.getChildren())
      {
         if (item.getValue() instanceof PackageHeaderParameterTreeValue && item.getValue().toString().equals(text))
         {
            return item;
         }
      }
      return null;
   }
}
