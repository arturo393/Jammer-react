# Changelog

Todos los cambios notables de este proyecto serán documentados en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/es/1.0.0/),
y este proyecto adhiere a [Versionado Semántico](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - Rama optimization

### Agregado

- Función F() para mejorar la estabilidad del sistema
- Comando `memtest` para revisar el estado de la memoria
- Código para lectura de memoria del sistema

### Arreglado

- Mejoras en la estabilidad general del sistema
- Corrección de problemas con el corta corriente que se quedaba pegado
- Se quitó el timer de la memoria para mayor estabilidad
- Mejoras para evitar que el sistema se quede bloqueado

### En pruebas

- La función F() aún está en fase de pruebas
- Persisten algunos problemas con el corta corriente

---

## [v2.1] - 2018-12-18

### Nuevas características

- Función de bloquear dispositivo vía Bluetooth
- Variable no volátil para el estado del jammer
- ID del dispositivo por Bluetooth

### Mejoras

- Gestión mejorada del estado del jammer

---

## [v2.0] - 2018-11-29

### Cambios importantes

- **CAMBIO MAYOR**: Reestructuración completa del código
- Nueva arquitectura del sistema más robusta

### Nuevas características

- Lectura y escritura de caracteres por Bluetooth
- Protocolo de comunicación mejorado

---

## [v1.9.0] - 2018-08-21

### Correcciones críticas

- **CRÍTICO**: Se eliminan prints que causaban mal funcionamiento del programa
- Corrección de errores en interrupciones
- Cambio de `vPrintStringAndNumber` por `Serial.print` para evitar errores

### Nuevas características

- Manejo mejorado de interrupciones
- Configuración de prioridades de tareas

---

## [v1.1] - 2018-02-20

### Nuevas características

- Variables de estado para relay de corta-corriente
- Sensor de puerta configurable
- Conexión directa del corta-corriente sin relay
- Delay para estabilización de salidas de pines

### Removido

- Bibliotecas innecesarias
- Cambio de estado en señal keep alive (siempre activo ahora)

---

## [v1.0] - 2017-12-01

### Nuevas características

- Distribución de pines actualizada
- Archivos GERBER para fabricación
- LED indicador durante alerta de jamming
- Configuración correcta de pines para la placa
- Serial.print para monitoreo de puerta
- Corrección de salidas para corta corriente

### Hardware

- Diseño inicial de PCB
- Esquemáticos del circuito
- Simulación en SPICE

---

## Versiones anteriores (2017-07 a 2017-11)

### Desarrollo inicial

- Implementación del protocolo básico de anti-jamming
- Integración con FreeRTOS
- Soporte para iButton
- Desarrollo del hardware inicial
- Configuración del entorno de desarrollo
- Implementación de bibliotecas base

---

## Protocolo de Software

### Funcionamiento básico

1. **Monitoreo continuo**: Se revisa constantemente la entrada JAMIN
2. **Detección de jamming**: Si JAMIN == LOW:
   - JAMOUT = LOW por 90 segundos
   - DOUT intercambia entre LOW y abierto durante 90 segundos
   - Se entra en modo alarma durante 90 segundos
3. **Activación por puerta**: Si durante el modo alarma DOORIN == Abierto:
   - JAMOUT = LOW durante 5 minutos después que termine el modo alarma

### Características técnicas

- Compatible con dispositivos Teltonika FMA120
- Comunicación vía Bluetooth
- Soporte para sensores de puerta
- Sistema de corta corriente configurable
- Notificaciones cada 10 minutos
- Desbloqueo por GPS
- Apagado automático programable
