# Virtual Arm Realtime

Servicio mínimo para calcular la posición de la broca en cada dato recibido del UM982.

- `use_imu = false`: brazo 2D fijo, usa posición ANT1 y heading. No espera que la
  máquina se detenga ni que la IMU se estabilice.
- `use_imu = true`: rota el brazo completo con pitch UM982 y roll del
  acelerómetro (MPU6050 o el MPU9250 del GY-91; se detecta solo y se
  publica en `imu.chip`). Cambiar de placa obliga a rehacer «Detectar
  ejes» y el cero: el mapeo y las taras describen el sensor anterior.
- Salida del cliente: `GET /api/v1/telemetry/stream` en SSE, compatible con `gpsevt`.
  El estado se republica cada 500 ms aunque no lleguen tramas, para que una
  pantalla congelada se delate por la edad del dato.
- Interfaz compacta en `http://<IP-RPi>:8080/` para NTRIP, CRS, brazo, modo y
  calibración 2D.
- No hay base de datos, historial, OpenAPI ni endpoints de diagnóstico.

## Límites que el servicio impone

- El brazo no puede exceder ±20 m, ni al cargar el archivo ni como resultado de
  una calibración. Un brazo mayor sólo sale de un CRS equivocado o de un dígito
  perdido al teclear el punto.
- El CRS debe ser una proyección en metros. Los geográficos (`WGS84`,
  `EPSG:4326`) se rechazan: PROJ los acepta y el Este/Norte pasarían a grados.
- La calibración promedia una ventana de 60 s con la máquina detenida
  (velocidad ≤ 0.05 m/s) y devuelve la dispersión de las soluciones. Diez épocas
  a 10 Hz son un segundo de datos y no reducen el sesgo RTK.
- En modo 2D el servicio publica el error horizontal que descarta por el pitch
  del UM982 (`attitude.tilt_error_m`) y avisa a partir de 5 cm. Es una cota
  inferior: el roll sin IMU es desconocido.
- El roll no se publica si no se puede sostener: si el eje que el mapeo llama
  vertical no es el que aguanta la gravedad (`max_tilt_deg`), o si dispersa más
  de `max_roll_noise_deg` con la máquina detenida, `roll_deg` queda en nulo y en
  3D no hay posición. Un roll equivocado no se ve en pantalla —mueve la broca en
  silencio— y `down_m·sin(roll)` convierte cada grado en centímetros.
- Con la máquina en marcha sólo se integra el giróscopo. El acelerómetro mide
  fuerza específica: una frenada o un bamboleo se leerían como ladeo.

La interfaz usa únicamente:

- `GET/PUT /api/config`: parámetros operativos; nunca devuelve la contraseña.
- `POST /api/calibration/2d`: calcula adelante/izquierda promediando una ventana
  de 60 s con la máquina detenida.
- `POST /api/gnss/port`: suelta o recupera `/dev/serial0` sin reiniciar el
  servicio, para el relevo con `virtual-rtk` sin entrar por SSH. Al liberar
  también se corta el NTRIP. El estado **no se persiste**: reiniciar el servicio
  vuelve siempre al puerto conectado.
- `GET /api/v1/telemetry/stream`: estado y posición en tiempo real.

Los cambios se guardan de forma atómica y se aplican en caliente. Cambiar sólo el
brazo, el CRS o el modo no cierra el puerto GNSS; cambiar NTRIP reconecta únicamente
la sesión NTRIP.

## Ejecución local de pruebas

```bash
python -m venv .venv
.venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q
```

## Producción

1. Ejecutar `sudo bash scripts/install.sh`; instala pero no inicia el servicio.
2. Confirmar `forward_m` y `left_m` contra un punto topográfico independiente.
3. Mantener `use_imu = false` hasta resolver la altura ANT1–broca. `down_m` se
   puede anotar desde la interfaz sin salir de 2D: no afecta a Este/Norte, pero
   sí a la cota publicada de la broca en los dos modos.
4. Detener el servicio que tenga el UART antes de iniciar éste: `virtual-rtk`,
   `gpsgo` o el que esté corriendo. Sólo uno puede abrir `/dev/serial0`.

El cambio de modo es una sola línea en `config.toml` y requiere reiniciar el servicio.
No existe conmutación automática para evitar saltos silenciosos de posición.

Activación, después de validar el brazo:

```bash
sudo systemctl stop gpsgo
sudo systemctl enable --now virtual-arm-realtime
```

Reversión inmediata:

```bash
sudo systemctl disable --now virtual-arm-realtime
sudo systemctl start gpsgo
```

El servicio escucha siempre en **8080**. `gpsevt` (del cliente) lee la ruta que
diga su `.env` en `/home/pi/Documents/dtsmine/backend/`, que ha ido cambiando
(8080 → 8081 → 8082); ese archivo es suyo y no se toca. Si quiere consumir este
servicio, su `.env` debe apuntar a
`http://localhost:8080/api/v1/telemetry/stream`. Comprobar ese `.env` antes de cambiar el puerto: no es un valor fijo.
