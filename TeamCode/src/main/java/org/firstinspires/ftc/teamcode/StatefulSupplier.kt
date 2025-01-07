package org.firstinspires.ftc.teamcode

class StatefulSupplier<S, T>(
    private val stateToSupplierMap: Map<S, () -> T>,
    initialState: S
) {
    private var currentState: S = initialState

    fun get(): T {
        val supplier = stateToSupplierMap[currentState] ?: throw IllegalArgumentException("No supplier for state: $currentState")
        return supplier()
    }

    fun setState(state: S) {
        if (!stateToSupplierMap.containsKey(state)) throw IllegalArgumentException("No supplier for state: $state")
        currentState = state
    }

    fun getState(): S = currentState
}