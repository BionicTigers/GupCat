package io.github.bionictigers.io.github.bionictigers.axiom.web

@Retention(AnnotationRetention.RUNTIME)
@Target(AnnotationTarget.PROPERTY, AnnotationTarget.FIELD)
annotation class Display(val name: String = "", val priority: Int = 0)
