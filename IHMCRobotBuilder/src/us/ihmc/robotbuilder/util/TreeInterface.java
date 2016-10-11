package us.ihmc.robotbuilder.util;

import javaslang.Function2;

import java.util.List;
import java.util.Optional;
import java.util.WeakHashMap;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

import static javaslang.collection.List.empty;

/**
 * Generic interface for a tree structure with support methods such as mapping or filtering.
 */
public interface TreeInterface<T extends TreeInterface> {
    /**
     * Get the child nodes of this tree node
     * @return child nodes
     */
    Iterable<T> getChildren();

    /**
     * Returns a lazy stream of child nodes of this tree node. A convenience method around {@link #getChildren()}.
     * @return children as {@link Stream}
     */
    default Stream<T> childStream() {
        return StreamSupport.stream(getChildren().spliterator(), false);
    }

    /**
     * Flattens the given tree to a sequential stream of nodes.
     * @param tree tree to flatten
     * @param <T> tree type
     * @return flattened tree
     */
    static <T extends TreeInterface<T>> Stream<T> flatten(T tree) {
        //noinspection Convert2MethodRef
        return Stream.concat(Stream.of(tree), tree.childStream().flatMap(child -> flatten(child)));
    }

    /**
     * Maps one tree to another, keeping the parent -> child relationships in the mapped tree.
     * The mapper is required to create target tree nodes based on a source node and a list of
     * already mapped children.
     * @param tree tree to map to another
     * @param mapper function that maps source nodes to target nodes
     * @param <Source> source tree type
     * @param <Target> target tree type
     * @return mapped tree root
     */
    static <Source extends TreeInterface<Source>, Target> Target map(Source tree, final TreeNodeMapper<? super Source, Target> mapper) {
        List<Target> mappedChildren = tree.childStream()
                .map(child -> map(child, mapper))
                .collect(Collectors.toList());

        return mapper.mapNode(tree, mappedChildren);
    }

    /**
     * Creates a cached/memoized mapping function that does not re-evaluate
     * nodes that have been mapped in the past.
     * @param mapper mapping function
     * @param <Source> source tree type
     * @param <Target> target tree type
     * @return cached map function
     */
    static <Source extends TreeInterface<Source>, Target> CachedMapper<Source, Target> cachedMap(final TreeNodeMapper<? super Source, Target> mapper) {
        return new CachedMapper<>(mapper);
    }

    /**
     * Removes nodes (and their children) that do not match the predicate from the tree.
     * Returns a new, pruned tree.
     * @param predicate filter predicate
     * @return filtered tree
     */
    static <T extends TreeInterface<T>> Optional<T> filter(T tree, Predicate<? super T> predicate, TreeNodeSupplier<T> nodeSupplier) {
        if (predicate.test(tree)) {
            List<T> filteredChildren = tree.childStream()
                    .map(child -> filter(child, predicate, nodeSupplier))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .collect(Collectors.toList());
            T filteredNode = nodeSupplier.supply(tree, filteredChildren);
            return Optional.of(filteredNode);
        } else {
            return Optional.empty();
        }
    }

    interface TreeNodeMapper<Source extends TreeInterface, Target> {
        Target mapNode(Source node, List<Target> children);
    }

    interface TreeNodeSupplier<T extends TreeInterface> {
        T supply(T source, List<T> children);
    }

    class CachedMapper<Source extends TreeInterface<Source>, Target> {
        private final Cache<Source, Target> cache = new Cache<>(Integer.MAX_VALUE, WeakHashMap<Source, Cache<Source, Target>.CachedItem>::new);
        private final TreeNodeMapper<? super Source, Target> mapper;

        private CachedMapper(TreeNodeMapper<? super Source, Target> mapper) {
            this.mapper = mapper;
        }

        public Target map(Source node) {
            return cache.getItem(node).orElseGet(() -> {
                List<Target> children = node.childStream().map(this::map).collect(Collectors.toList());
                Target mapped = mapper.mapNode(node, children);
                cache.cacheItem(node, mapped);
                return mapped;
            });
        }

        public long getCacheMisses() {
            return cache.getMisses();
        }

        public long getCacheHits() {
            return cache.getHits();
        }
    }
}
