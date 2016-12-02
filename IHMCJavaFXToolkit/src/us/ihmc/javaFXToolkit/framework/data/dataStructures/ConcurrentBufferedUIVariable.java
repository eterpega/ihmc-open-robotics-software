package us.ihmc.javaFXToolkit.framework.data.dataStructures;

import us.ihmc.javaFXToolkit.framework.data.UIVariable;
import us.ihmc.javaFXToolkit.framework.data.UIVariableType;

/**
 *
 * In ConcurrentBufferedUIVariable buffer access should be thread safe: Implementation made with write lock and read locks.
 *
 * This implementation uses a ring buffer of double primitive type.
 *
 * Created by amoucheboeuf on 12/16/15.
 */
public class ConcurrentBufferedUIVariable<T> extends AbstractBufferedUIVariable
{
   /**
    *
    * @param uiVariable
    * @param capacity
    */
   public ConcurrentBufferedUIVariable(UIVariable<T> uiVariable, int capacity)
   {
      super(uiVariable, new ConcurrentRingDoubleBuffer(capacity), capacity); // gets the value
   }
}
