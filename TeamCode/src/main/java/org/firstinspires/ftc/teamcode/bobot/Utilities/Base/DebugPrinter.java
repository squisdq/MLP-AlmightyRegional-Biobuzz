package org.firstinspires.ftc.teamcode.bobot.Utilities.Base;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class DebugPrinter {
    private final List<Object> owners = new ArrayList<>();
    private final List<Field> fields = new ArrayList<>();

    public DebugPrinter(Object... targets) {
        for (Object target : targets)
            for (Field field : target.getClass().getDeclaredFields())
                if (field.isAnnotationPresent(Debug.class)) {
                    field.setAccessible(true);
                    owners.add(target);
                    fields.add(field);
                }
    }

    public void print(Telemetry telemetry) {
        for (int i = 0; i < fields.size(); i++)
            try {
                Field field = fields.get(i);
                String name = field.getAnnotation(Debug.class).value();
                telemetry.addData(name.isEmpty() ? field.getName() : name, field.get(owners.get(i)));
            } catch (IllegalAccessException ignored) {}
    }
}