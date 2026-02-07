package frc.robot.utils;

import java.util.LinkedList;

// An extention of LinkedList that limits the max number of elements when using add and remove.
public class LimitedQueue<E> extends LinkedList<E> {
    private int limit;

    public LimitedQueue(int limit) {
        this.limit = limit;
    }

    @Override
    public boolean add(E o) {
        super.add(o);
        while (size() > limit) { super.remove(); }
        return true;
    }
}
