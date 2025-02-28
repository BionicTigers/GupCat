package io.github.bionictigers.io.github.bionictigers.axiom.utils

import kotlin.reflect.KClass
import kotlin.reflect.full.memberProperties
import kotlin.reflect.full.superclasses

/**
 * Non-inline recursive helper that searches through the supertypes
 * for a property named [propertyName] that is annotated with [annotationClass].
 */
fun <T : Annotation> searchSuperTypes(
    kClass: KClass<*>,
    propertyName: String,
    annotationClass: KClass<T>
): Boolean {
    for (supertype in kClass.superclasses) {
        val superProp = supertype.memberProperties.firstOrNull { it.name == propertyName }
        if (superProp != null && superProp.annotations.any { annotationClass.isInstance(it) }) {
            return true
        }
        if (searchSuperTypes(supertype, propertyName, annotationClass)) {
            return true
        }
    }
    return false
}

/**
 * Inline function that checks if the property named [propertyName] on the given [instance]
 * is annotated with annotation type T. This function first checks the property declared in the class,
 * then delegates to [searchSuperTypes] to check the supertypes.
 */
inline fun <reified T : Annotation> hasAnnotationOnProperty(
    instance: Any,
    propertyName: String
): Boolean {
    val kClass = instance::class
    // Check if the property declared on this class has the annotation.
    val property = kClass.memberProperties.firstOrNull { it.name == propertyName }
    if (property != null && property.annotations.any { it is T }) {
        return true
    }
    // Delegate to the non-inline recursive helper for supertypes.
    return searchSuperTypes(kClass, propertyName, T::class)
}

fun <T : Any> String.convertTo(targetClass: KClass<T>): T {
    return when (targetClass) {
        Int::class -> this.toInt() as T
        Double::class -> this.toDouble() as T
        Float::class -> this.toFloat() as T
        Long::class -> this.toLong() as T
        Boolean::class -> this.toBoolean() as T
        String::class -> this as T
        else -> throw IllegalArgumentException("Unsupported conversion to ${targetClass.simpleName}")
    }
}

