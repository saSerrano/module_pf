# Módulo Básico: Seguimiento de Personas
Este repositorio contiene la implementación del módulo básico de seguimiento de personas. 

## Dependencias
Para que éste módulo pueda funcionar apropiadamente es necesario que se los siguientes nodos se encuentren ejecutando:
- homer_mapping
- map_manager_h
- homer_navigation

todos pertenecientes a los paquetes ```homer_mapping```, ```homer_map_manager``` y ```homer_navigation```.

## Ejecución
El módulo se puede echar a andar con dos archivos launch.
- ```bm.launch```: Levanta solamente al módulo basico, de manera que está listo para operar y comunicarse con el nodo maestro.
- ```open_doors_demo.launch```: Hace todo lo que el archivo ```bm.launch``` y además levanta un nodo que sirve como traductor entre  comandos emitidos por el nodo Alexa-ROS y el módulo básico. Este nodo adicional se utilizó en el demo para ele evento de Puertas Abiertas 2022.

## Futuras Mejoras
El estado actual del módulo le permite realizar el seguimiento de una persona en condiciones sencillas: cuando terceros no se atraviesan entre la persona y el robot, solo hay una persona a la que seguir, la persona es bastante paciente y esperará al robot en caso de quedarse muy atrás.
Por esto, a continuación se enlistan algunas mejoras con las cuales el módulo incrementaría en robustez frente a condiciones adversas:
- **Identificación de Personas**: Actualmente, si el módulo pierde de viste a la persona que sigue, comenzará a seguir a la siguiente persona que entre en su campo visual, sin importar  quien sea. Hace falta añadir la habilidad de que construir un modelo de la persona que está siguiendo para poder reindentificarla en caso de perderla de vista.
- **Búsqueda del camino más corto**: Para seguir a una persona, el robot recorre el mismo camino recorrido por la persona. Esta política causa que el robot se tarde más de lo necesario en alcanzar a la persona cuando ésta sigue trayectoriasen forma de "U". Por esto, el módulo se beneficiaría de una abilidad para estar buscar de manera constante el camino libre de colisiones más corto entre el robot y la persona seguida.
- **Seguimiento Avanzado de Personas**: Si una persona se atraviesa entre la cámara del robot y la persona seguida, el robot comenzará a seguir a la persona que se atravesó en su camino. Para evitar este error, el módulo requiere de un método de seguimiento más sofisticado en el que se tenga una hipótesis de donde se encuentre la persona seguida, incluso en los instantes en los que no la pueda ver.
- **Búsqueda Activa de Persona**: Cuando el robot pierde de vista a la persona que sigue, éste se queda quieto esperando que la persona regrese a su campo visual. El proceso de reidentificación de la persona sería más robusto y menos tardado si el robot fuera proactivo en la búsqueda de dicha persona, tal vez mirando a sus alrededores, o recorriendo la vecindad inmediata del último lugar en el que se vió a la persona.