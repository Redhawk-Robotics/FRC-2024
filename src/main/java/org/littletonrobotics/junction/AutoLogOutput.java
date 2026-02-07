package org.littletonrobotics.junction;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Minimal annotation stub for @AutoLogOutput to allow compilation without AdvantageKit. */
@Target({ElementType.METHOD, ElementType.TYPE})
@Retention(RetentionPolicy.RUNTIME)
public @interface AutoLogOutput {
  // key used by the real AdvantageKit annotation. Keep optional for stubbing.
  String key() default "";
}
