# Cardtastic: análisis y propuesta para v0.7

## 1. Resumen

La base de v0.6 ya cubre el objetivo principal: conectar un Cardputer a una
radio Meshtastic, listar nodos y canales, intercambiar mensajes y guardar
historiales en microSD. La separación entre cliente Meshtastic y aplicación
permite evolucionarla sin reescribirla.

Recomiendo que **v0.7 priorice estabilidad, integridad del historial y claridad
de la mensajería**. Hay problemas concretos en esas áreas que conviene resolver
antes de ampliar mucho las funciones.

Este documento es una propuesta de trabajo. Los identificadores `CT-01`, etc.,
son referencias de planificación, no números de issues de GitHub. Las ramas
son nombres sugeridos; se crearían al comenzar cada tarea.

## 2. Alcance y evidencia de la revisión

- Base revisada: `main`, commit `caf9abc` (documentación de v0.6).
- Revisados: `src/main.cpp`, `src/MeshtasticClient.cpp`,
  `src/MeshtasticClient.h`, configuración PlatformIO, README, changelog,
  estructura versionada y definiciones protobuf relevantes.
- Compilación local: `pio run -e m5stack-cardputer`, correcta con las
  dependencias ya instaladas. PlatformIO se ejecutó desde
  `~/.platformio/penv/bin/pio` porque no está en el PATH del entorno.
- Dependencias resueltas en esa compilación: M5Cardputer
  `1.1.1+sha.e023280`, NimBLE-Arduino `2.3.7`, Nanopb `0.4.91`,
  ArduinoJson `6.21.5`, Arduino ESP32 `2.0.16`.
- Uso informado: RAM estática 36.920 / 327.680 bytes (11,3 %);
  flash 830.625 / 3.342.336 bytes (24,9 %).
- Estos porcentajes no miden el máximo de memoria dinámica: conversaciones,
  nodos, documentos JSON y líneas de chat se asignan durante la ejecución.
- No se realizaron pruebas físicas, ni una instalación desde cero de todas
  las dependencias. Una compilación correcta no valida BLE, SD ni entrega
  por radio.
- `test/` contiene solamente el README de ejemplo; no hay pruebas propias
  versionadas. Tampoco hay workflows de GitHub Actions versionados.
- No aparecen etiquetas en el repositorio local. No se consultaron releases
  ni issues remotos: `gh` no está instalado en este entorno.

Clasificación de hallazgos:

- **Confirmado por código:** la implementación contiene el defecto descrito;
  eso no significa que se haya reproducido físicamente su consecuencia.
- **Riesgo por validar:** hay un escenario plausible que requiere simulación,
  trazas o prueba con hardware/protocolo.
- **Mejora:** capacidad nueva o cambio de diseño, sin afirmar un fallo actual.

## 3. Arquitectura actual

| Área | Implementación | Observación |
| --- | --- | --- |
| BLE y protocolo | `MeshtasticClient.cpp`, 1.665 líneas; cabecera de 230 líneas | Escaneo, autenticación, GATT, protobuf, nodos, canales y seguimiento de entrega juntos. |
| Aplicación | `main.cpp`, 3.086 líneas | Modelos, SD, codificación de texto, teclado, pantallas y bucle principal en un mismo archivo. |
| Pantallas | `Screen`, `ScreenManager` y nueve pantallas | Buena base para extraer componentes de a poco. |
| Mensajes | Vectores de conversaciones y hasta 50 mensajes por conversación | Hay límite por conversación, pero no global de conversaciones ni de memoria. |
| Persistencia | JSON en `/cardtastic` | Identidades separadas por radio; archivos de conversación compartidos entre radios. |
| Preferencias | NVS mediante `Preferences` | Auto-conexión y última radio. |
| Protocolo | C++ generado en `lib/meshtastic-protobuf` | Falta documentar procedencia y regeneración. |
| Compilación | PlatformIO, ESP32-S3 | Plataforma fijada; M5Cardputer y NimBLE no están fijadas a revisiones exactas. |

Aspectos positivos a conservar:

- Conexión BLE asíncrona y captura del tipo de dirección obtenido del escaneo.
- Cola de observaciones de escaneo protegida con sección crítica.
- Separación entre texto original y transformación para mostrarlo.
- Identificadores de nodo tratados como `uint32_t` en la persistencia.
- Seguimiento de entrega mediante tokens y estados independientes del texto.
- Manejo de microSD opcional y soporte de funcionamiento en RAM.
- README breve y útil para compilar, conectar y operar el equipo.

## 4. Problemas y tareas propuestas

Prioridades:

- **P1:** corregir antes de cerrar v0.7 por estabilidad o integridad de datos.
- **P2:** recomendable; ajustar al tamaño que se quiera para la versión.
- **P3:** backlog de funciones y evolución.

Tamaño relativo: **S** acotado, **M** varios componentes/casos, **L** requiere
diseño y validación amplia. No son estimaciones de días.

### CT-01 — Corregir inserción ordenada de canales

- **Tipo:** bug confirmado. **Prioridad:** P1. **Tamaño:** S.
- **Evidencia:** `MeshtasticClient.cpp:519–533`, `getOrCreateChannel()`.
  Se llama `_channels.insert(it, entry)` y después se desreferencia `it`.
  La inserción invalida ese iterador, incluso sin realocar, porque está en
  la posición de inserción. Usarlo después es comportamiento indefinido.
- **Impacto:** corrupción o acceso inválido si los canales llegan fuera de
  orden. Que normalmente lleguen ordenados puede ocultar el defecto.
- **Alcance:** usar el iterador devuelto por `insert`, revisar duplicados y
  mantener la lista ordenada.
- **Aceptación:** secuencias 2, 0, 1 y canales repetidos producen una lista
  única ordenada; el canal devuelto es el correcto; prueba de regresión.
- **Rama:** `fix/channel-insertion`.
- **Dependencias:** ninguna.

### CT-02 — Hacer finita y verificable la sincronización inicial

- **Tipo:** bugs confirmados. **Prioridad:** P1. **Tamaño:** M.
- **Evidencia:** `MeshtasticClient.cpp:261–281, 1247–1279, 1460–1467`.
  Tras cada escritura exitosa de `requestConfig()`, `_configAttempts` vuelve
  a cero. Si la radio acepta solicitudes pero no termina de responder,
  los timeouts no agotan el límite previsto. Además, se acepta cualquier
  `config_complete_id`, sin cotejarlo con la solicitud vigente.
- **Impacto:** permanecer indefinidamente en SYNC o dar por completa una
  solicitud equivocada. El protobuf documenta explícitamente el ID como
  protección frente a configuraciones antiguas/parciales.
- **Alcance:** contador por sesión, ID esperado, estados distinguibles de
  autenticación/sincronización/listo/error y acción de reintento.
- **Aceptación:** los intentos se agotan en un tiempo acotado; una respuesta
  antigua no completa la sincronización; se informa el fallo y se puede
  reintentar; definir y verificar qué datos mínimos habilitan el envío.
- **Rama:** `fix/config-sync-lifecycle`.
- **Dependencias:** coordinar con CT-03.

### CT-03 — Procesar eventos BLE en un único contexto

- **Tipo:** acceso concurrente sin protección confirmado; consecuencias a
  reproducir. **Prioridad:** P1. **Tamaño:** L.
- **Evidencia:** callbacks `MeshClientCallbacks` invocan directamente
  `handleConnect()`, `handleDisconnect()` y otros manejadores. Estos modifican
  `String`, vectores, colas, estado y punteros GATT que también utiliza el
  bucle Arduino. Por ejemplo, `resetConnectionState()` limpia estructuras
  mientras el bucle puede estar leyéndolas. La protección del escaneo no
  se extiende a estos eventos.
- **Impacto:** carreras y estados inconsistentes durante desconexiones,
  autenticación, cambios de radio o lecturas GATT.
- **Alcance:** callbacks breves que publican eventos con datos copiados;
  procesamiento de estado y modelos desde el bucle principal. Definir
  capacidad, desbordamiento y orden de eventos; evitar bloqueos GATT en
  secciones críticas.
- **Aceptación:** eventos tardíos de una conexión anterior no afectan a la
  nueva; conectar/desconectar repetidamente no invalida modelos en uso;
  la UI permanece operativa y las trazas permiten reconstruir la sesión.
- **Rama:** `fix/ble-event-ownership`.
- **Dependencias:** base para CT-04, CT-08 y CT-16.

### CT-04 — Aislar el estado de cada sesión y cada radio

- **Tipo:** defecto de aislamiento confirmado; escenarios a probar.
  **Prioridad:** P1. **Tamaño:** M.
- **Evidencia:** `main.cpp:772–786` reinicia las conversaciones al cambiar de
  radio solamente después de conseguir almacenamiento. Sin SD, el cambio
  de radio conserva conversaciones en RAM de la anterior. El chat activo y
  la selección de nodo tampoco están ligados a una sesión estable.
- **Impacto:** mostrar o utilizar conversaciones y selecciones de otra radio.
  Durante una desconexión un chat abierto también puede seguir intentando
  guardar cuando `myNodeNum()` ya vale cero.
- **Alcance:** identidad de radio/sesión independiente de SD; transición
  explícita de chat, selección de nodo, cachés y mensajes pendientes.
  Referenciar nodos por número estable, no solo por índice de vector.
- **Aceptación:** alternar A/B con SD y sin SD no mezcla estados; una
  selección anterior no apunta a otro nodo tras reconectar; nunca se guarda
  un historial atribuyéndolo a una radio desconocida.
- **Rama:** `fix/radio-session-isolation`.
- **Dependencias:** CT-03; coordinar formato con CT-05.

### CT-05 — Separar archivos de conversación por radio

- **Tipo:** pérdida de historial posible, confirmada por rutas.
  **Prioridad:** P1. **Tamaño:** M.
- **Evidencia:** `main.cpp:626–631`. Los archivos son `ch_0.json`,
  `dm_<peer>.json`, etc., para todas las radios. El campo `radio` se comprueba
  al cargar, pero no evita que al guardar desde B se reemplace el archivo
  previamente escrito por A.
- **Alcance:** espacio de almacenamiento por radio, por ejemplo
  `/cardtastic/<radio>/...`; migración compatible de archivos v1.
- **Aceptación:** dos radios conservan sus historiales del mismo canal y
  destinatario; volver a A recupera A; migrar no destruye el archivo fuente
  antes de verificar el resultado; los archivos sin radio tienen una
  política explícita de atribución.
- **Rama:** `fix/history-radio-namespace`.
- **Dependencias:** acordar identidad con CT-04.

### CT-06 — Hacer robustas las escrituras y recuperación de microSD

- **Tipo:** defectos de manejo de errores confirmados. **Prioridad:** P1.
  **Tamaño:** L.
- **Evidencia:** `persistConversation()` elimina el archivo anterior antes de
  escribir el nuevo, no comprueba `doc.overflowed()` ni el resultado de
  escritura. La caché de identidades usa un temporal, pero elimina el
  original antes de renombrarlo y tampoco verifica escritura completa.
  Las cargas pueden marcarse completadas aunque haya fallado una lectura.
- **Impacto:** pérdida o truncamiento de historial por SD llena, retirada,
  fallo de escritura o apagado. El formato temporal por sí solo no garantiza
  atomicidad en FAT ni ante pérdida de energía.
- **Alcance:** escritura verificada, temporal/backup y recuperación al iniciar;
  errores observables, reintentos con espera y estado dirty. Considerar el
  caso de insertar una SD cuando ya hay mensajes en RAM: hoy un historial
  no vacío en RAM impide cargar el del disco y puede acabar reemplazándolo.
- **Aceptación:** fallos inyectados en escritura/rename no destruyen la última
  copia válida; SD llena/retirada no bloquea el chat; los historiales de RAM
  y SD se reconcilian con una política documentada; no se reintenta escribir
  sin pausa en cada iteración tras un error persistente.
- **Rama:** `fix/storage-recovery`.
- **Dependencias:** CT-05; CT-09 ayuda a reconciliar mensajes.

### CT-07 — Evitar truncamiento silencioso al enviar texto

- **Tipo:** bug confirmado. **Prioridad:** P1. **Tamaño:** M.
- **Evidencia:** `sendTextMessage()` recorta al tamaño del payload protobuf
  (233 bytes), mientras `ChatScreen` permite escribir sin límite y guarda el
  texto completo como enviado. El historial puede diferir de lo transmitido;
  el recorte también puede dividir una secuencia UTF-8.
- **Alcance acordado:** máximo de producto de 200 bytes UTF-8 compartido entre
  editor y cliente. Es deliberadamente conservador respecto de los 233 bytes
  del campo protobuf: el firmware añade sobrecarga de serialización y puede
  agregar cifrado PKI. Rechazo explícito sin truncamiento, contador en bytes,
  ventana horizontal que conserve visible el cursor al final y borrador
  editable ante cualquier fallo inmediato. El límite no depende de si una
  versión futura del firmware amplía su payload.
- **Compatibilidad comprobada por fuente:** el teclado clásico de Meshtastic
  v2.6.11 permite hasta `DATA_PAYLOAD_LEN` en su editor, mientras las apps
  oficiales Android y Apple usan 200 bytes. Cardtastic conserva el límite de
  200 como política común para v2.6.11 y firmware posteriores.
- **Aceptación:** probar límite − 1, límite y límite + 1; el cliente nunca
  recorta silenciosamente; el mensaje registrado coincide con el payload; el
  final del borrador sigue visible; fallos de validación/escritura no crean
  un mensaje enviado ni borran el borrador.
- **Rama:** `fix/message-byte-limit`.
- **Dependencias:** ninguna.

### CT-08 — Precisar y preservar estados de entrega

- **Tipo:** limitaciones confirmadas y correlación por validar.
  **Prioridad:** P1. **Tamaño:** L.
- **Evidencia:** `handleQueueStatus()` asigna IDs a envíos pendientes por FIFO;
  `resetConnectionState()` no termina ni aísla los pendientes de la sesión
  anterior. Todos los timeouts se convierten en FAILED. `persistConversation()`
  no guarda estado de entrega y `applyDeliveryUpdate()` solo actualiza RAM.
- **Impacto:** incertidumbre al reconectar o con varios envíos, pérdida de
  estados al reiniciar y confusión entre fallo explícito y falta de ACK.
- **Alcance:** investigar correlación QueueStatus/packet ID en versiones
  soportadas, modelar sesión y destinatario, separar error explícito de
  entrega no confirmada e informar causa. Persistir resultados conocidos;
  no restaurar un pendiente antiguo como si siguiera rastreándose.
- **Aceptación:** DMs y broadcasts con éxito, error de cola, NAK, timeout,
  envíos consecutivos y desconexión tienen estados coherentes; un ACK viejo
  no confirma otro mensaje; los estados finales sobreviven al reinicio;
  explicar que un ACK no equivale a lectura por una persona.
- **Rama:** `fix/delivery-state-lifecycle`.
- **Dependencias:** CT-03, CT-04 y CT-06.

### CT-09 — Conservar ID y hora de mensajes; deduplicar recepción

- **Tipo:** mejora de modelo y riesgo de duplicados por validar.
  **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** al convertir `MeshPacket` a `MeshTextMessage` se pierden ID
  y hora. La aplicación agrega todos los textos sin deduplicación. No se
  puede distinguir un eco del envío local de un mensaje nuevo mediante ID.
- **Alcance:** conservar identificadores, origen, canal y metadatos temporales;
  deduplicación acotada por radio/sesión y estrategia para reconciliar eco
  local con QueueStatus. No deduplicar solo por texto.
- **Aceptación:** el mismo paquete repetido aparece una vez; dos mensajes
  distintos con igual texto se conservan; las horas desconocidas se muestran
  como desconocidas; la política admite reinicios y migración del historial.
- **Rama:** `feat/message-metadata`.
- **Dependencias:** coordinar con CT-05, CT-06 y CT-08.

### CT-10 — Mostrar actividad real de los nodos

- **Tipo:** error semántico confirmado. **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** al importar `NodeInfo` se fija `lastUpdateMs = millis()`
  aunque `last_heard` sea antiguo; la UI calcula ACTIVE/STALE y el orden
  usando `lastUpdateMs`. Además, enviar un DM llama `noteNode(dest)` y hace
  aparecer reciente un destino del que no se recibió nada.
- **Impacto:** nodos antiguos parecen activos durante los primeros cinco
  minutos; orden de sincronización se confunde con recencia de radio.
- **Alcance:** separar recepción local de metadatos y última actividad real;
  usar tiempo de radio/paquete cuando sea válido, con referencia temporal
  explícita y fallback honesto cuando no lo sea.
- **Aceptación:** sincronizar un nodo antiguo no lo vuelve activo; enviarle
  un mensaje tampoco; recibir tráfico sí actualiza actividad; el orden
  refleja la recencia y se distingue desconocido de cero.
- **Rama:** `fix/node-activity-semantics`.
- **Dependencias:** CT-09 recomendable para unificar tiempo.

### CT-11 — Actualizar pantallas y no leídos sin depender del teclado

- **Tipo:** problemas de actualización y no leídos confirmados.
  **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** el cambio de conexión solo invalida HOME/CONNECT;
  actualizaciones de nodos sin cambio de identidad, batería y edad no
  fuerzan un repintado. Un mensaje en el chat activo no marca no leído,
  aunque la pantalla esté dormida. La lista de conversaciones reordena
  elementos sin conservar la selección por identidad.
- **Alcance:** eventos/invalidation por pantalla; refresco periódico acotado
  para datos temporales, selección estable y política de lectura basada en
  visibilidad real. Separar actividad del dispositivo de estado de pantalla.
- **Aceptación:** nodos, batería y conexión se actualizan sin pulsar teclas;
  recibir mensajes no cambia qué conversación seleccionó el usuario;
  mensajes con pantalla dormida quedan pendientes de leer; no hay repintado
  continuo innecesario. Comprobar físicamente sueño/despertar y parpadeo.
- **Rama:** `fix/ui-live-state`.
- **Dependencias:** CT-03, CT-04 y CT-10.

### CT-12 — Mantener operativa la UI durante escaneo y sincronización

- **Tipo:** bloqueo estructural confirmado; impacto a medir.
  **Prioridad:** P2. **Tamaño:** L.
- **Evidencia:** `startScan()` mantiene el control hasta tres intentos de
  ocho segundos, con callbacks que dibujan pero sin procesar teclado.
  `pullFromRadioUntilEmpty()` no tiene presupuesto de tiempo/paquetes.
  `debugDumpServices()` descubre todos los servicios en cada conexión.
- **Alcance:** escaneo por estados, cancelación, recepción por lotes con
  presupuesto y diagnóstico GATT bajo opción de depuración.
- **Aceptación:** se puede cancelar escaneo y volver al menú; la sincronización
  de una base grande no impide usar teclado; no se pierden paquetes por
  repartir trabajo entre iteraciones; medir latencia antes y después.
- **Rama:** `perf/nonblocking-ble-work`.
- **Dependencias:** CT-03 y CT-02.

### CT-13 — Reproducir la compilación y verificarla en GitHub

- **Tipo:** mejora de mantenimiento. **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** M5Cardputer apunta a Git sin revisión fija, NimBLE no tiene
  versión fijada y no hay CI versionada. `platformio.ini` referencia un
  archivo local ignorado: hay que comprobar un clon sin ese archivo, no
  asumir que el comportamiento del equipo actual lo valida.
- **Alcance:** fijar revisiones compatibles, probar clon limpio sin overrides,
  documentar herramientas y agregar workflow de compilación en PR/main.
  Evaluar separar flags de diagnóstico y release; hoy `CORE_DEBUG_LEVEL=5`.
- **Aceptación:** el checkout limpio compila sin puertos ni archivos privados;
  CI publica resultado y tamaños; actualizar dependencias es un cambio
  explícito y revisable. No mezclar esta tarea con una actualización general
  de framework sin motivo.
- **Rama:** `ci/reproducible-build`.
- **Dependencias:** ninguna.

### CT-14 — Extraer lógica testeable y agregar regresiones selectivas

- **Tipo:** mejora de arquitectura y calidad. **Prioridad:** P2. **Tamaño:** L.
- **Alcance:** extraer incrementalmente modelos de conversación, persistencia,
  texto y estados de entrega/sincronización de `main.cpp`. Mantener pantallas
  y hardware detrás de interfaces pequeñas cuando ayude a probar lógica.
  No hace falta imponer un framework grande ni una reescritura de UI.
- **Aceptación:** pruebas significativas para reintentos finitos, canales
  fuera de orden, límites de bytes, aislamiento de radios, migración y fallos
  de guardado. Fixtures protobuf/JSON versionadas y ejecutables en CI.
- **Rama:** `refactor/testable-core` como primera extracción; después ramas
  pequeñas por componente si el alcance crece.
- **Dependencias:** CT-13 para CI; las regresiones pequeñas deben acompañar
  los fixes correspondientes sin esperar a completar esta extracción.

### CT-15 — Estandarizar versión y publicación de firmware

- **Tipo:** mejora de releases. **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** versión escrita manualmente en `HomeScreen`, README y
  changelog. El script local `scripts/make_merged.sh` está ignorado por Git;
  por tanto, no es parte reproducible del proyecto compartido.
- **Alcance:** una fuente de versión para firmware, procedimiento de release,
  etiqueta `v0.7` (o acordar `v0.7.0`), binarios y offsets documentados,
  checksums y matriz mínima de pruebas. Revisar el script local con su autor
  antes de convertirlo en herramienta mantenida.
- **Aceptación:** firmware muestra versión inequívoca; etiqueta apunta al
  commit probado; release incluye instrucciones para el artefacto exacto;
  otro equipo puede generar ese artefacto desde el repositorio.
- **Rama:** `build/release-workflow`.
- **Dependencias:** CT-13; cierre de los fixes elegidos para v0.7.

### CT-16 — Reconectar tras pérdida de enlace y recuperar emparejamiento

- **Tipo:** mejora; auto-conexión actual limitada al arranque.
  **Prioridad:** P3. **Tamaño:** L.
- **Alcance:** reconexión con backoff después de pérdida involuntaria,
  cancelación explícita, acción para olvidar radio/bond y timeout claro de
  autenticación manual. Validar PIN completo de seis dígitos y presentar
  errores comprensibles. Distinguir desconexión del usuario de caída de radio.
- **Aceptación:** apagar/encender o alejar/acercar radio recupera enlace sin
  un bucle agresivo; desconectar manualmente no reconecta automáticamente;
  cambiar credenciales se puede resolver desde la UI sin borrar todo NVS.
- **Rama:** `feat/connection-recovery`.
- **Dependencias:** CT-02, CT-03, CT-04 y CT-08.

### CT-17 — Borradores por conversación y escritura en español

- **Tipo:** mejora UX. **Prioridad:** P3. **Tamaño:** M.
- **Evidencia:** `ChatScreen` tiene un único `inputBuffer` para todas las
  conversaciones y solo acepta ASCII imprimible. Renderizar tildes recibidas
  no equivale a poder escribirlas. Al cambiar de chat el borrador se arrastra.
- **Alcance:** borrador por identidad de conversación; combinaciones de teclado
  para español si se acuerdan, borrado UTF-8 por carácter y edición visible.
- **Aceptación:** cambiar A/B recupera cada borrador; cambiar radio no los
  mezcla; borrar una tilde no deja bytes inválidos; documentar las teclas.
- **Rama:** `feat/chat-drafts-and-input`.
- **Dependencias:** CT-04 y CT-07.

### CT-18 — Consultar y gestionar historiales sin enlace BLE

- **Tipo:** mejora funcional. **Prioridad:** P3. **Tamaño:** M.
- **Evidencia:** Conversations exige `myNodeNum() != 0` para mostrar la lista,
  aunque haya historiales en SD. No hay gestión de borrado desde la UI.
- **Alcance:** elegir la radio/historial guardado, leer offline, borrar o
  archivar conversaciones de forma explícita y mostrar estado SD/RAM.
- **Aceptación:** se consulta historial sin radio; la identidad de su radio
  está visible; no se intenta enviar hasta tener una sesión compatible;
  borrar respeta el espacio de cada radio.
- **Rama:** `feat/offline-history`.
- **Dependencias:** CT-04, CT-05 y CT-06.

### CT-19 — Notificar mensajes y ofrecer ajustes básicos

- **Tipo:** mejora funcional. **Prioridad:** P3. **Tamaño:** M.
- **Alcance:** aviso sonoro opcional, política configurable de despertar
  pantalla, brillo y tiempo de apagado, con preferencias persistentes.
- **Aceptación:** avisos desactivables; no interrumpen PIN o envío; canal
  silenciado tiene un significado definido; ajustes sobreviven al reinicio.
- **Rama:** `feat/message-notifications` (y ajustes en PR separado si crece).
- **Dependencias:** CT-11.

### CT-20 — Precisar canales y compatibilidad Meshtastic

- **Tipo:** mezcla de defecto de nombre confirmado y compatibilidad por validar.
  **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** canal primario sin nombre se presenta como `LongFast` de
  manera fija; la configuración LoRa no se procesa para derivar esa etiqueta.
  Canales deshabilitados siguen siendo abribles. Los DMs nuevos se crean en
  canal 0, y una conversación DM existente no incorpora el canal a su identidad.
  Debe contrastarse el comportamiento esperado con el protocolo soportado.
- **Alcance:** etiqueta neutral o derivada del preset real, disponibilidad de
  canales, política explícita para canal de DM y tabla de versiones/firmwares
  probados. Documentar origen y revisión de protobuf, generación y nanopb.
  Verificar tráfico PKI y errores de incompatibilidad antes de prometer soporte.
- **Aceptación:** un preset distinto no se llama erróneamente LongFast;
  canal deshabilitado no ofrece envío operativo; DMs y canales secundarios
  funcionan en la matriz elegida; se sabe regenerar los protobuf incluidos.
- **Rama:** `fix/channel-and-protocol-compatibility`.
- **Dependencias:** CT-01, CT-02; coordinar modelo de conversación con CT-05.

### CT-21 — Acotar memoria y trabajo para redes e historiales grandes

- **Tipo:** riesgo de escalabilidad por validar. **Prioridad:** P2. **Tamaño:** M.
- **Evidencia:** no hay límite global de conversaciones/nodos/cola RX;
  `loadConversationsFromStorage()` recorre y carga todos los archivos.
  Cada repintado reconstruye todas las líneas de los 50 mensajes del chat,
  incluso cuando solo cambió un carácter del borrador. La caché de identidades
  se recorre en cada iteración y puede escribirse muchas veces durante SYNC.
- **Alcance:** medir mínimo de heap, mayor bloque y latencia; establecer
  presupuestos, carga diferida o expulsión si las mediciones lo justifican;
  agrupar escrituras de caché y cachear layout de texto con invalidación clara.
- **Aceptación:** prueba con cantidades documentadas de nodos, conversaciones
  y mensajes máximos; memoria y colas acotadas, desbordamiento observable;
  una sincronización grande no genera escritura de SD por cada nodo.
- **Rama:** `perf/bounded-memory-and-layout`.
- **Dependencias:** CT-06, CT-09, CT-12.

### CT-22 — Mejorar información de nodos y navegación

- **Tipo:** mejora funcional. **Prioridad:** P3. **Tamaño:** M.
- **Alcance:** nombre largo y corto, edad legible en lugar de epoch crudo,
  filtros/búsqueda por nombre o ID, y lectura de telemetría/posición si se
  acuerda como objetivo. Hoy esos paquetes no actualizan métricas específicas
  después del NodeInfo inicial. Ajustar ayudas al ancho de 240 píxeles y
  documentar colores de entrega y atajo A de auto-conexión.
- **Aceptación:** distinguir métrica ausente de valor cero; batería y posición
  muestran antigüedad; selección estable al filtrar; ayudas caben en pantalla.
- **Rama:** `feat/node-details-and-search` (dividir telemetría si se incluye).
- **Dependencias:** CT-10 y CT-11.

## 5. Alcance recomendado de v0.7

### Núcleo recomendado

1. CT-01: inserción de canales.
2. CT-02: sincronización finita y correlacionada.
3. CT-03: propiedad y procesamiento de eventos BLE.
4. CT-04: aislamiento de sesiones.
5. CT-05: historial separado por radio.
6. CT-06: guardado y recuperación SD.
7. CT-07: límite de mensaje sin pérdida silenciosa.
8. CT-08: estados de entrega coherentes.
9. CT-13: compilación reproducible y CI.
10. CT-15: procedimiento de release.

CT-09 puede entrar parcialmente como habilitador de reconciliación y entrega.
Las regresiones de CT-14 deben viajar con los fixes, aunque la modularización
completa quede para después. Es un alcance sustancial: si se busca una versión
rápida, conviene dividir el ciclo en entregas pequeñas, sin llamar estable a
una combinación que aún no tenga resueltos los problemas de historial elegidos.

### Siguiente prioridad si hay margen

CT-10 y CT-11 (actividad e interfaz), CT-12 (respuesta durante BLE), CT-20
(canales/compatibilidad) y CT-21 (mediciones y límites). El defecto puntual
del nombre LongFast puede resolverse en un PR pequeño antes del resto de CT-20.

### Backlog funcional

CT-16 a CT-19 y CT-22. Elegir una mejora visible, por ejemplo borradores o
notificaciones, puede ser útil; no es necesario comprometerlas todas en v0.7.

### Orden de ejecución sugerido

1. CI/reproducibilidad y fixes acotados: CT-13, CT-01, CT-07.
2. Eventos/sesión/sincronización: CT-03, CT-02, CT-04.
3. Identidad de almacenamiento y recuperación: CT-05, metadatos necesarios
   de CT-09, CT-06.
4. Entrega y persistencia de estado: CT-08.
5. Mejoras elegidas de actividad, UI y rendimiento; pruebas integradas.
6. Cierre, documentación y publicación: CT-15.

No abrir todas las ramas por adelantado: cada una debe partir del `main`
actualizado cuando se empieza su tarea. Si un issue L no cabe en un PR
revisable, descomponerlo en issues hijos con resultados verificables.

## 6. Validación necesaria

### Automatizable

- Compilación limpia y comprobación de tamaño del firmware.
- Canales desordenados, IDs `uint32_t` altos y nodos inexistentes.
- Sincronización con timeout, respuestas antiguas y agotamiento de intentos.
- Payload máximo, texto UTF-8 y borradores demasiado largos.
- Aislamiento de radios, migración JSON y recuperación de archivos incompletos.
- Deduplicación y correlación de estados con fixtures del protocolo.

### En Cardputer y radios reales

- Primer emparejamiento, bond existente, PIN incorrecto y radio no disponible.
- Desconexión durante lectura GATT, autenticación, sincronización y envío.
- Cambiar de radio con/sin SD; reiniciar y recuperar historiales de ambas.
- Canal primario/secundario y DM: éxito, falta de ACK y rechazo explícito.
- Varios mensajes consecutivos y eco del propio envío.
- SD ausente, llena, retirada y reinserción; interrupción de alimentación
  controlada durante guardado con datos de prueba recuperables.
- Mensajes con pantalla dormida, despertar, no leídos y batería.
- Red con muchos nodos, historial grande y sesión prolongada.

Registrar versiones de hardware, firmware Meshtastic y dependencias. Afirmar
compatibilidad con lo realmente probado, sin extrapolar a todos los modelos.

## 7. Organización en GitHub

1. Revisar esta propuesta y elegir el alcance de `v0.7`.
2. Crear milestone `v0.7` y issues solo con tareas acordadas. El resto puede
   quedar en backlog sin milestone.
3. Cada issue debería contener: problema/objetivo, evidencia o reproducción,
   alcance, criterios de aceptación, validación y dependencias.
4. Etiquetas sugeridas: `bug`, `enhancement`, `maintenance`, `priority:high`,
   `priority:medium`, y áreas `ble`, `messaging`, `storage`, `ui`, `build`.
   No es necesario crear etiquetas que no se vayan a usar.
5. Una rama corta por tarea y PR hacia `main`, enlazando el issue con
   `Closes #N`. Mantener `main` compilable y probado al integrar.
6. Crear ramas cuando se trabaja en ellas; eliminarlas después del merge.
7. Al cerrar la versión, actualizar changelog/documentación, identificar el
   commit probado con una etiqueta y publicar los artefactos.

No hace falta una rama permanente `develop` ni una rama `v0.7` para este
proyecto. Una rama de release solo aportaría valor si hubiera que estabilizar
esa versión mientras `main` avanza con cambios de una versión posterior.

## 8. Decisiones a tomar antes de crear los issues

- Confirmar el alcance mínimo y si v0.7 será principalmente de estabilización.
- Elegir versiones Meshtastic y radios de la matriz inicial de pruebas.
- Acordar migración de los historiales existentes y qué hacer con archivos
  antiguos sin identidad de radio.
- Elegir cómo expresar entrega no confirmada y qué estados conservar.
- Elegir, si se desea, una función visible del backlog para esta versión.

Con esas decisiones se pueden convertir las fichas anteriores en issues
concretos sin comprometer de golpe todo el backlog.
