package com.ma5951.utils.Utils;

import java.util.function.Supplier;

public class BooleanLatch {
    private boolean state;
    private Supplier<Boolean> booleanSupplier;

    public BooleanLatch(Supplier<Boolean> BooleanSupplier) {
        booleanSupplier = BooleanSupplier;
        this.state = false;
    }

    public void set() {
        this.state = true;
    }

    public void reset() {
        this.state = false;
    }

    public boolean get() {
        if (booleanSupplier.get()) {
            state = true;
        }
        return this.state;
    }

}
