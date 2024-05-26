package org.firstinspires.ftc.teamcode.centerstage.util.Input;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.FIELD)
@Retention(RetentionPolicy.RUNTIME)
/**
 * Toggles of the KeyCode type are updated automatically if the UpdateAutomatically annotation is present and there's no need to call the update function on them
 */
public @interface UpdateAutomatically {
}
