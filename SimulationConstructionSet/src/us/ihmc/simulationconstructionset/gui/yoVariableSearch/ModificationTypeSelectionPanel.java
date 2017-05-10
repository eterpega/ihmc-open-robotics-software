package us.ihmc.simulationconstructionset.gui.yoVariableSearch;

import java.awt.Color;
import java.awt.GridLayout;
import java.util.EnumMap;

import javax.swing.JCheckBox;
import javax.swing.JPanel;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.dataStructures.variable.VariableModificationType;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class ModificationTypeSelectionPanel extends JPanel
{
   private static final long serialVersionUID = -1360596353537620660L;

   private final JCheckBox isEnabled;
   private final EnumMap<VariableModificationType, JCheckBox> typeEnabled = new EnumMap<>(VariableModificationType.class);

   public ModificationTypeSelectionPanel()
   {
      super(new GridLayout(VariableModificationType.values().length + 1, 1));

      isEnabled = new JCheckBox("Enable", true);
      this.add(isEnabled);

      for (VariableModificationType modificationType : VariableModificationType.values())
      {
         String name = "Show " + StringUtils.capitalize(modificationType.name().toLowerCase());
         JCheckBox checkBox = new JCheckBox(name, modificationType.isEnabledByDefault());
         typeEnabled.put(modificationType, checkBox);
         this.add(checkBox);
      }
   }

   public boolean isVariableModifiable(YoVariable<?> yoVariable)
   {
      if (!isEnabled.isSelected())
         return true;

      return yoVariable.getModificationType().getIsModifiable();
   }

   public Color getColor(YoVariable<?> yoVariable)
   {
      if (!isEnabled.isSelected())
         return Color.BLACK;

      return yoVariable.getModificationType().getTextColor();
   }

   public boolean isActive(YoVariable<?> variable)
   {
      if (!isEnabled.isSelected())
         return true;

      return typeEnabled.get(variable.getModificationType()).isSelected();
   }
}
