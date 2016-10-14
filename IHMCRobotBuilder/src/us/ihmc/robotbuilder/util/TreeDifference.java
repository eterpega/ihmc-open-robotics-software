package us.ihmc.robotbuilder.util;

import javaslang.Tuple;
import javaslang.Tuple2;
import javaslang.collection.*;

import java.util.Optional;

/**
 *
 */
public class TreeDifference
{

   /**
    * Computers a difference between two {@link Tree}s. The tree values and nodes
    * are compared using reference equals (==) to guarantee good runtime complexity.
    * This algorithm does not guarantee to return the minimum edit distance (as that
    * is O(N^3)) but tries a few heuristics with O(N) runtime for common tree edits
    * and O(N^2) worst case runtime.
    * @param original original tree
    * @param otherTree new tree
    * @param <T> tree value type
    * @return tree {@link Difference}s grouped by tree nodes they apply to
    */
   public static <T> DifferencesByNode<T> difference(Tree<T> original, Tree<T> otherTree)
   {
      return new DifferencesByNode<>(HashMap.ofEntries(differenceImpl(original, otherTree)));
   }

   private static <T> Seq<Tuple2<Tree<T>, Difference<T>>> differenceImpl(Tree<T> original, Tree<T> otherTree)
   {
      if (original == otherTree)
         return List.empty();

      Map<Tree<T>, Integer> myIndex = indexChildren(original);
      Map<Tree<T>, Integer> otherIndex = indexChildren(otherTree);

      Set<Tree<T>> addedChildren = otherIndex.keySet().filter(otherChild -> !myIndex.containsKey(otherChild));
      Set<Tree<T>> removedChildren = myIndex.keySet().filter(myChild -> !otherIndex.containsKey(myChild));

      Map<Tree<T>, Tree<T>> replacedChildren = findReplacedChildren(addedChildren, removedChildren);
      Set<Tree<T>> replacements = replacedChildren.values().toSet();
      addedChildren = addedChildren.filter(child -> !replacements.contains(child));
      removedChildren = removedChildren.filter(child -> !replacedChildren.containsKey(child));

      Difference<T> difference = new Difference<>(otherTree, addedChildren, removedChildren, otherIndex);
      return replacedChildren
                  .flatMap(replacement -> differenceImpl(replacement._1, replacement._2))
                  .prepend(Tuple.of(original, difference));
   }

   /**
    * Grouped {@link Difference}s by node. A thin wrapper around a
    * hash map to avoid extra long type specification in client code.
    * @param <T> tree value type
    */
   public static class DifferencesByNode<T>
   {
      private final Map<Tree<T>, Difference<T>> differencesByNode;

      DifferencesByNode(Map<Tree<T>, Difference<T>> differencesByNode)
      {
         this.differencesByNode = differencesByNode;
      }

      public Optional<Difference<T>> get(Tree<T> node)
      {
         return differencesByNode.get(node).toJavaOptional();
      }

      public Map<Tree<T>, Difference<T>> getAllDifferences()
      {
         return differencesByNode;
      }
   }

   /**
    *
    * @param <T>
    */
   public static class Difference<T> {
      private final Tree<T> newNode;
      private final Set<Tree<T>> addedChildren;
      private final Set<Tree<T>> removedChildren;
      private final Map<Tree<T>, Integer> finalChildOrder;

      Difference(Tree<T> newNode, Set<Tree<T>> addedChildren, Set<Tree<T>> removedChildren, Map<Tree<T>, Integer> finalChildOrder)
      {
         this.newNode = newNode;
         this.addedChildren = addedChildren;
         this.removedChildren = removedChildren;
         this.finalChildOrder = finalChildOrder;
      }

      public Tree<T> getNewNode()
      {
         return newNode;
      }

      public Set<Tree<T>> getAddedChildren()
      {
         return addedChildren;
      }

      public Set<Tree<T>> getRemovedChildren()
      {
         return removedChildren;
      }

      public Map<Tree<T>, Integer> getFinalChildOrder()
      {
         return finalChildOrder;
      }
   }

   private static <T> Map<Tree<T>, Integer> indexChildren(Tree<T> tree)
   {
      return HashMap.ofEntries(tree.children().zipWithIndex()).mapValues(x -> (int)(long)x);
   }

   private static <T> Map<Tree<T>, Tree<T>> findReplacedChildren(Set<Tree<T>> addedChildren, Set<Tree<T>> removedChildren)
   {
      List<Tuple2<Tree<T>, Tree<T>>> result = List.empty();
      Set<Tree<T>> remainingChildren = addedChildren;
      for (Tree<T> removedChild : removedChildren)
      {
         if (remainingChildren.isEmpty())
            break;
         Tree<T> replacement = findClosestTree(removedChild, remainingChildren);
         result = result.prepend(Tuple.of(removedChild, replacement));
         remainingChildren = remainingChildren.remove(replacement);
      }
      return HashMap.ofEntries(result);
   }

   private static <T> Tree<T> findClosestTree(Tree<T> base, Set<Tree<T>> others)
   {
      return others.map(tree -> new ScoredTree<>(tree, computeChildSimilarity(base, tree)))
                   .reduce((tree1, tree2) -> tree1.score > tree2.score ? tree1 : tree2)
                   .tree;
   }

   private static <T> int computeChildSimilarity(Tree<T> tree1, Tree<T> tree2)
   {
      return tree1.children()
                  .zip(tree2.children())
                  .count(childPair -> childPair._1.getValue() == childPair._2.getValue());
   }

   private static class ScoredTree<T> {
      private final Tree<T> tree;
      private final int score;

      private ScoredTree(Tree<T> tree, int score)
      {
         this.tree = tree;
         this.score = score;
      }
   }
}
