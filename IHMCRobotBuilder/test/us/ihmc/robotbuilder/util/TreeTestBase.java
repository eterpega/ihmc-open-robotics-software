package us.ihmc.robotbuilder.util;

import javaslang.collection.List;

/**
 *
 */
public class TreeTestBase
{
   static final Tree<Integer> BINARY_TREE = tree(0, tree(1), tree(2));
   static final Tree<Integer> SINGLETON_TREE = tree(0);
   static final Tree<Integer> DEEP_TREE = tree(0, tree(1, tree(3), tree(4), tree(5)), tree(2));

   private static Tree<Integer> tree(int value, Tree... children)
   {
      //noinspection unchecked
      return new Tree<>(value, List.of((Tree<Integer>[])children));
   }
}
