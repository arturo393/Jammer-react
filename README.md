# Jammer-react
Sistem designed complement the AntiJammer of the Teltonika FMA120 devices.



## Software Protocol

1. Constantemente se revisa JAMIN.
2. If JAMIN == LOW.
 2.1 JAMOUNT  = LOW. por 90 (s)
 2.2 DOUT intercambia entre LOW y abierto duranta 90 (s).
 2.3 Se entra en modo alarma durante 90 (s)
3. Si durante el modo alarma DOORIN == Abierto
 3.1 JAMOUT = LOW durante5 minutos despues que termine el modo alarma




## Documentation
1. Flow diagram of the software.
2. Bitácora
3. 
