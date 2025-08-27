# 🚨 Jammer-React

Sistema de complemento anti-jamming diseñado para dispositivos Teltonika FMA120, con arquitectura basada en FreeRTOS y comunicación Bluetooth.

[![License](https://img.shields.io/github/license/arturo393/Jammer-react)](LICENSE)
[![Version](https://img.shields.io/badge/version-v2.1-blue)](CHANGELOG.md)
[![Status](https://img.shields.io/badge/status-optimization-yellow)](RELEASE_NOTES.md)

## 📋 Descripción

El sistema Jammer-React es un dispositivo de seguridad vehicular que detecta y responde a intentos de jamming de señales GPS/GSM. Está diseñado para trabajar como complemento de los dispositivos Teltonika FMA120, proporcionando una capa adicional de protección contra interferencias maliciosas.

### 🎯 Características Principales

- **Detección automática de jamming** en tiempo real
- **Sistema de corta corriente** configurable 
- **Comunicación Bluetooth** para configuración remota
- **Sensor de puerta** integrado
- **Monitoreo de memoria** con comando de diagnóstico
- **LED indicador** de estados del sistema
- **Compatible con FreeRTOS**

## 🔧 Protocolo de Funcionamiento

### Algoritmo Principal

1. **Monitoreo Continuo**: Se revisa constantemente la entrada `JAMIN`
2. **Detección de Jamming**: Si `JAMIN == LOW`:
   - `JAMOUT = LOW` por 90 segundos
   - `DOUT` intercambia entre LOW y abierto durante 90 segundos
   - Se entra en **modo alarma** durante 90 segundos
3. **Activación por Puerta**: Si durante el modo alarma `DOORIN == Abierto`:
   - `JAMOUT = LOW` durante 5 minutos después que termine el modo alarma

### Estados del Sistema

- **Normal**: Monitoreo continuo sin detección de jamming
- **Alarma**: Jamming detectado, activación de contramedidas
- **Puerta Abierta**: Extensión del modo de protección

## 🏗️ Estructura del Proyecto

```
Jammer-react/
├── Hardware/              # Diseños de PCB y esquemáticos
│   ├── v1/               # Versión 1 del hardware
│   └── v2/               # Versión 2 del hardware
├── JammerRTOS/           # Código principal con FreeRTOS
├── jammerv2/             # Versión 2 del firmware
├── jammerv3/             # Versión 3 del firmware (sin FreeRTOS)
├── Simulation/           # Simulaciones SPICE
├── case/                 # Archivos para carcasa
├── logos/                # Recursos gráficos
├── CHANGELOG.md          # Historial de cambios
├── RELEASE_NOTES.md      # Notas de la versión actual
└── README.md            # Este archivo
```

## 🚀 Instalación y Configuración

### Prerrequisitos

- **PlatformIO** instalado
- **Arduino IDE** (opcional)
- Microcontrolador compatible con **ESP32/Arduino**
- Dispositivo **Teltonika FMA120**

### Configuración del Hardware

1. Conectar las entradas según el esquemático en `/Hardware/v2/`
2. Configurar los pines según la definición en el código
3. Verificar las conexiones de alimentación

### Compilación

```bash
# Usando PlatformIO
cd JammerRTOS
pio run

# Para upload
pio run --target upload
```

## 📊 Versiones

### Actual: v2.1 (Rama optimization)
- ✅ Función F() para mejora de estabilidad
- ✅ Comando `memtest` para diagnóstico
- ✅ Sistema de lectura de memoria mejorado
- 🔄 En pruebas: Optimizaciones del corta corriente

### Historial de Versiones

- **v2.1** (2018-12-18): Funciones Bluetooth y estado no volátil
- **v2.0** (2018-11-29): Reestructuración completa del código
- **v1.9.0** (2018-08-21): Correcciones críticas de estabilidad
- **v1.1** (2018-02-20): Variables de estado y sensores
- **v1.0** (2017-12-01): Versión inicial estable

Ver [CHANGELOG.md](CHANGELOG.md) para detalles completos.

## 🛠️ Configuración via Bluetooth

El sistema permite configuración remota via Bluetooth:

- **ID del dispositivo**: Configuración única por unidad
- **Estados del sistema**: Monitoreo en tiempo real
- **Parámetros de funcionamiento**: Ajuste de tiempos y umbrales
- **Comando memtest**: Diagnóstico de memoria

## 🔍 Diagnóstico y Debug

### Comando memtest
```
memtest
```
Proporciona información sobre:
- Uso actual de memoria
- Memoria disponible
- Estado de las tareas FreeRTOS

### LEDs Indicadores
- **LED Principal**: Estado general del sistema
- **Durante Alarma**: Parpadeo característico
- **Modo Configuración**: Patrón específico

## 📈 Especificaciones Técnicas

| Característica | Especificación |
|----------------|----------------|
| Microcontrolador | ESP32/Arduino compatible |
| Voltaje de operación | 12V DC |
| Consumo | <100mA en standby |
| Temperatura | -20°C a +60°C |
| Comunicación | Bluetooth 2.0/4.0 |
| Entradas digitales | 3 (JAMIN, DOORIN, GPS) |
| Salidas digitales | 2 (JAMOUT, DOUT) |

## 🤝 Contribución

1. Fork el proyecto
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📝 Licencia

Este proyecto está bajo la licencia especificada en el archivo [LICENSE](LICENSE).

## 👨‍💻 Autor

**Arturo Veras** - [@arturo393](https://github.com/arturo393)

## 📞 Soporte

Para soporte técnico o preguntas:
- Abre un [issue](https://github.com/arturo393/Jammer-react/issues)
- Consulta la documentación en [RELEASE_NOTES.md](RELEASE_NOTES.md)

---

⚠️ **Nota**: Este sistema está diseñado para uso legítimo de seguridad vehicular. Asegúrate de cumplir con las regulaciones locales sobre dispositivos de radiofrecuencia. 
