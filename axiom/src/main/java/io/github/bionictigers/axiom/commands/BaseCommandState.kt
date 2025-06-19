package io.github.bionictigers.axiom.commands

import io.github.bionictigers.axiom.web.Hidden
import kotlin.time.Duration
import kotlin.time.TimeMark
import kotlin.time.TimeSource

open class BaseCommandState(
    @Hidden var enteredAt: TimeMark?,
    @Hidden var lastExecutedAt: TimeMark?,
    var timeInScheduler: Duration = Duration.ZERO,
    var deltaTime: Duration = Duration.ZERO,
) {
    fun resetTimings() {
        enteredAt = null
        lastExecutedAt = null
        timeInScheduler = Duration.ZERO
        deltaTime = Duration.ZERO
    }
}