package us.ihmc.robotics.util;

import javaslang.Function2;
import org.junit.Test;
import us.ihmc.robotics.util.TreeInterface;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.junit.Assert.*;
import static us.ihmc.robotics.util.TreeInterface.*;

public class TreeInterfaceTest
{
    static class SimpleTree implements TreeInterface<SimpleTree>
    {
        private final List<SimpleTree> children;

        SimpleTree(SimpleTree... children) {
            this(Arrays.asList(children));
        }

        SimpleTree(List<? extends SimpleTree> children) {
            this.children = new ArrayList<>();
            this.children.addAll(children);
        }

        @Override
        public Iterable<SimpleTree> getChildren() {
            return children;
        }
    }

    private static class MappedTree implements TreeInterface<MappedTree>
    {
        private List<MappedTree> children;
        private String name;

        MappedTree(String name, List<MappedTree> children) {
            this.children = children;
            this.name = name;
        }

        String getName() {
            return name;
        }

        @Override
        public Iterable<MappedTree> getChildren() {
            return children;
        }
    }

    private static final SimpleTree SINGLE_NODE = new SimpleTree();
    private static final SimpleTree ONE_CHILD = new SimpleTree(new SimpleTree());
    private static final SimpleTree TWO_CHILDREN = new SimpleTree(new SimpleTree(), new SimpleTree());
    private static final SimpleTree TWO_LAYERS = new SimpleTree(new SimpleTree(), TWO_CHILDREN);

    @Test
    public void testFlattenReturnsAllNodes() {
        assertStreamEquals(flatten(SINGLE_NODE), Stream.of(SINGLE_NODE));
        assertStreamEquals(flatten(ONE_CHILD), Stream.of(ONE_CHILD, ONE_CHILD.children.get(0)));
        assertStreamEquals(flatten(TWO_LAYERS), Stream.of(TWO_LAYERS, TWO_LAYERS.children.get(0), TWO_CHILDREN, TWO_CHILDREN.children.get(0), TWO_CHILDREN.children.get(1)));
    }

    @Test
    public void testMapMapsAllNodes() {
        testMapFunctionMapsAllNodes(TreeInterface::map);
    }

    @Test
    public void testCachedMapMapsAllNodes() {
        testMapFunctionMapsAllNodes((Function2<SimpleTree, TreeNodeMapper<SimpleTree, MappedTree>, MappedTree>) (simpleTree, mapper) -> {
            CachedMapper<SimpleTree, MappedTree> cached = cachedMap(mapper);
            return cached.map(simpleTree);
        });
    }

    private void testMapFunctionMapsAllNodes(Function2<SimpleTree, TreeNodeMapper<SimpleTree, MappedTree>, MappedTree> mapFn) {
        MappedTree single = mapFn.apply(SINGLE_NODE, (node, children) -> new MappedTree("a", children));
        assertEquals(single.name, "a");


        AtomicInteger counter = new AtomicInteger(), counter2 = new AtomicInteger();
        MappedTree complex = mapFn.apply(TWO_LAYERS, (node, children) -> new MappedTree("node" + counter.incrementAndGet(), children));
        assertEquals(
                flatten(complex)
                        .map(MappedTree::getName)
                        .collect(Collectors.toSet()),
                flatten(TWO_LAYERS)
                        .map(node -> "node" + counter2.incrementAndGet())
                        .collect(Collectors.toSet()));
    }

    @Test
    public void testTrueFilterKeepsAllNodes() {
        Optional<SimpleTree> filtered = filter(TWO_LAYERS, node -> true, (node, children) -> new SimpleTree(children));
        assertTrue(filtered.isPresent());

        assertEquals(filtered.get().children.size(), TWO_LAYERS.children.size());
        assertEquals(filtered.get().children.get(0).children.size(), TWO_LAYERS.children.get(0).children.size());
    }

    @Test
    public void testFalseFilterRemovesEverything() {
        Optional<SimpleTree> filtered = filter(TWO_LAYERS, node -> false, (node, children) -> new SimpleTree(children));
        assertFalse(filtered.isPresent());
    }

    @Test
    public void testFilterRemovesSpecificNode() {
        Optional<SimpleTree> filtered = filter(TWO_LAYERS, node -> node != TWO_CHILDREN, (node, children) -> new SimpleTree(children));
        assertTrue(filtered.isPresent());

        assertEquals(filtered.get().children.size(), 1);
    }

    @Test
    public void testCachedMapCachesResults() {
        AtomicInteger mapCalls = new AtomicInteger(0);
        int expectedMapCalls = (int)flatten(TWO_LAYERS).count();
        CachedMapper<SimpleTree, MappedTree> mapFn = cachedMap((node, children) -> {
            mapCalls.incrementAndGet();
            return new MappedTree("node" + node.hashCode(), children);
        });
        MappedTree mapped = mapFn.map(TWO_LAYERS);
        MappedTree mappedAgain = mapFn.map(TWO_LAYERS);

        assertTrue(mapped == mappedAgain); // should return the same reference
        assertEquals(mapCalls.get(), expectedMapCalls);
        assertEquals(1, mapFn.getCacheHits());
        assertEquals(expectedMapCalls, mapFn.getCacheMisses()); // the first map goes uncached for each node
    }

    private static <T> void assertStreamEquals(Stream<T> a, Stream<T> b) {
        assertArrayEquals(a.toArray(), b.toArray());
    }
}
