# Quanser BTEC Qcar

# Proyecto de Seguimiento de Línea

Este proyecto consiste en un seguidor de línea que utiliza scripts en Python para controlar el funcionamiento del robot. A continuación, se detalla cómo configurar y ejecutar el demo.

## Ejecución del Demo

Para ejecutar el demo, sigue estos pasos:

1. **Otorgar Permisos de Ejecución a los Scripts:**

   Antes de ejecutar los scripts, es necesario otorgarles permisos de ejecución. Abre una terminal y ejecuta los siguientes comandos:

   ```bash
   chmod +x control_keys.py
   chmod +x LineFollower.py
   ```

Una vez que los scripts tienen permisos de ejecución, puedes ejecutarlos con el siguiente comando:

    sudo PYTHONPATH=$PYTHONPATH QAL_DIR=$QAL_DIR python3 control_keys.py

    sudo PYTHONPATH=$PYTHONPATH QAL_DIR=$QAL_DIR python3 LineFollower.py
