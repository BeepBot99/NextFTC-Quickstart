package org.firstinspires.ftc.teamcode

class StatefulSupplier<S, T>(
    private val stateToSupplierMap: Map<S, () -> T>,
    initialState: S
) {
    var state: S = initialState
        get() = field
        set(value) {
            if (!stateToSupplierMap.containsKey(value)) {
                throw IllegalArgumentException("No supplier for state: $value")
            }
            field = value
        }

    fun get(): T {
        val supplier = stateToSupplierMap[state] ?: throw IllegalArgumentException("No supplier for state: $state")
        return supplier()
    }
}
