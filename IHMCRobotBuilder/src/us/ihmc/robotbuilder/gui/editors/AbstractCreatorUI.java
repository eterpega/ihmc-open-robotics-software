package us.ihmc.robotbuilder.gui.editors;

import us.ihmc.robotbuilder.gui.Creator.CreatorUI;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Common implementation of {@link CreatorUI}.
 */
public class AbstractCreatorUI implements CreatorUI
{
   private final List<Supplier<Optional<Throwable>>> confirmListeners = new ArrayList<>();
   private final List<Runnable> cancelListeners = new ArrayList<>();

   @Override public void addCancelActionListener(Runnable cancelled)
   {
      cancelListeners.add(cancelled);
   }

   @Override public void addConfirmAndValidationActionListener(Supplier<Optional<Throwable>> confirmed)
   {
      confirmListeners.add(confirmed);
   }

   protected Optional<Throwable> fireConfirm()
   {
      return confirmListeners.stream().map(Supplier::get).filter(Optional::isPresent).map(Optional::get).findFirst();
   }

   protected void fireCancel()
   {
      cancelListeners.forEach(Runnable::run);
   }
}
