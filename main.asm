.include "m2560def.inc"

;-----------------------------------------------------------------------------
; EQU Definiciones (constantes y pines)
;-----------------------------------------------------------------------------
.equ TRIG_PIN = 4        ; Disparo (trigger) del sensor ultrasónico en PORTH4
.equ ECHO_PIN = 3        ; Eco (echo) del sensor ultrasónico en PORTH5 (ver nota)
.equ F_CPU    = 16000000 ; Reloj de 16 MHz

;-----------------------------------------------------------------------------
; Definiciones de registros
;-----------------------------------------------------------------------------
.def temp      = r16     ; Registro temporal general
.def temp2     = r17     ; Para retrasos y UART
.def durationL = r18     ; Byte bajo de la medición del Timer4
.def durationH = r19     ; Byte alto de la medición del Timer4

;-----------------------------------------------------------------------------
; Rutina de inicialización
;-----------------------------------------------------------------------------

.cseg
.org 0x0000
    rjmp inicio

inicio:
    ; Inicializar puntero de pila (Stack Pointer)
    ldi temp, high(RAMEND)
    out SPH, temp
    ldi temp, low(RAMEND)
    out SPL, temp
    clr r1

    ;-----------------------------------------
    ; Configurar UART a 9600 baudios
    ;-----------------------------------------
    ldi temp, 0
    sts UBRR0H, temp
    ldi temp, 103            ; 103 para 9600 baudios @ 16 MHz
    sts UBRR0L, temp
    ldi temp, (1 << TXEN0)   ; Habilitar transmisor
    sts UCSR0B, temp
    ldi temp, (1 << UCSZ01) | (1 << UCSZ00) ; 8 bits de datos, 1 bit de parada, sin paridad
    sts UCSR0C, temp

    ;-----------------------------------------
    ; Configurar pines del sensor ultrasónico en PORTH:
    ;   TRIG (PORTH4) como salida,
    ;   ECHO (PORTH5) como entrada (ver nota en ECHO_PIN).
    ;-----------------------------------------
    lds temp, DDRH
    ori temp, (1 << TRIG_PIN)      ; Establecer TRIG como salida
    andi temp, ~(1 << ECHO_PIN)    ; Asegurar que ECHO sea entrada
    sts DDRH, temp

    ; Asegurar TRIG en nivel bajo inicialmente
    lds temp, PORTH
    andi temp, ~(1 << TRIG_PIN)
    sts PORTH, temp

    ;-----------------------------------------
    ; Configurar Timer4 para mediciones (modo normal, detenido)
    ;-----------------------------------------
    clr temp
    sts TCCR4A, temp
    sts TCCR4B, temp
    sts TCNT4H, temp
    sts TCNT4L, temp

    ;-----------------------------------------
    ; Configurar motor:
    ;   - Salidas de dirección digitales en los 4 bits bajos de PORTB
    ;     (Pines digitales 50–53: avance = 1010, retroceso = 0101).
    ;   - PWM en Timer1 para habilitar motor en OC1A (PB5) y OC1B (PB6).
    ;
    ; Configurar DDRB para PB0–PB3 (dirección) y PB5–PB6 (PWM).
    ;-----------------------------------------
    ldi temp, (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3) | (1<<PB5) | (1<<PB6)
    out DDRB, temp

    ; Inicializar las salidas de dirección del motor (nibble bajo) a 0 (motor detenido)
    ldi temp, 0
    out PORTB, temp

    ;-----------------------------------------
    ; Configurar Timer1 para la señal PWM de habilitación de motor:
    ;   - Modo Fast PWM (modo 14), TOP = ICR1
    ;   - Modo no inversor en OC1A y OC1B
    ;   - Prescaler = 8
    ;-----------------------------------------
    ldi temp, (1<<WGM11) | (1<<COM1A1) | (1<<COM1B1)
    sts TCCR1A, temp
    ldi temp, (1<<WGM13) | (1<<WGM12) | (1<<CS11)
    sts TCCR1B, temp

    ; Ajustar ICR1 para un periodo de PWM (p.ej., ~20ms)
    ldi temp, high(21000)
    sts ICR1H, temp
    ldi temp, low(19000)
    sts ICR1L, temp

    ; Inicialmente, ciclo PWM = 0 (motor detenido)
    ldi temp, 0
    sts OCR1AH, temp
    sts OCR1AL, temp
    sts OCR1BH, temp
    sts OCR1BL, temp

    rjmp bucle_principal

;-----------------------------------------------------------------------------
; Bucle principal
;-----------------------------------------------------------------------------
bucle_principal:
    ;------------------------------------------------
    ; Medición con sensor ultrasónico usando Timer4:
    ;  1. Enviar un pulso de ~10µs en TRIG (PORTH4).
    ;  2. Esperar que ECHO (PORTH5) suba (alto) con timeout de 16 bits.
    ;  3. Borrar Timer4, iniciar Timer4 (prescaler=64).
    ;  4. Esperar que ECHO baje (bajo) con timeout de 16 bits.
    ;  5. Detener Timer4, leer el valor del contador y calcular la distancia.
    ;------------------------------------------------

    ; Enviar pulso de ~10 µs en TRIG
    lds temp, PORTH
    ori temp, (1 << TRIG_PIN)
    sts PORTH, temp
    ldi temp2, 40         ; Retardo aproximado para 10 µs @16MHz
    rcall retardo_us

    ; Poner TRIG en bajo
    lds temp, PORTH
    andi temp, ~(1 << TRIG_PIN)
    sts PORTH, temp

    ; -- Esperar que ECHO se ponga en alto con timeout 16 bits --
    ldi r30, 0xFF
    ldi r31, 0xFF
esperar_echo_alto:
    lds temp, PINH
    sbrs temp, ECHO_PIN   ; Si ECHO está en alto, saltar instrucción siguiente
    rjmp revision_echo_alto
    rjmp eco_alto_listo
revision_echo_alto:
    sbiw r30, 1           ; Decrementar contador de 16 bits (r31:r30)
    brne esperar_echo_alto
    rjmp sensor_timeout
eco_alto_listo:

    ; Borrar Timer4
    clr temp
    sts TCNT4H, temp
    sts TCNT4L, temp

    ; Iniciar Timer4 con prescaler=64
    ldi temp, (1<<CS41) | (1<<CS40)
    sts TCCR4B, temp

    ; -- Esperar que ECHO baje con timeout 16 bits --
    ldi r30, 0xFF
    ldi r31, 0xFF
esperar_echo_bajo:
    lds temp, PINH
    sbrs temp, ECHO_PIN
    rjmp eco_bajo_listo
    sbiw r30, 1
    brne esperar_echo_bajo
    rjmp sensor_timeout
eco_bajo_listo:

    ; Detener Timer4
    clr temp
    sts TCCR4B, temp

    ; Leer valor de Timer4 (byte bajo primero)
    lds durationL, TCNT4L
    lds durationH, TCNT4H

    ; Calcular distancia en cm
    rcall calcular_distancia   ; Resultado en r16
    mov r20, r16               ; Guardar distancia calculada en r20
    rjmp sensor_listo

sensor_timeout:
    ; En caso de timeout, fijar distancia=0 y detener Timer4
    clr r16
    clr temp
    sts TCCR4B, temp
sensor_listo:

    ;------------------------------------------------
    ; UART: Transmitir la distancia medida por serial
    ;------------------------------------------------
    mov r16, r20
    rcall enviar_distancia

    ;------------------------------------------------
    ; Nuevo control del motor con PWM basado en:
    ;   u = -150*(x - 30)
    ; donde x = distancia medida (r20), y la meta es 30 cm.
    ; Se define una banda muerta (±2 cm) en la que el motor se detiene.
    ;------------------------------------------------
    ldi   r17, 30      ; r17 = distancia objetivo (30 cm)
    mov   r21, r20     ; r21 = distancia medida
    sub   r21, r17     ; r21 = error = distancia - 30

    ; Verificar banda muerta y dirección:
    ; Si error >= 2, el robot está lejos -> avanzar
    cpi   r21, 2
    brge  control_avance

    ; Si error es negativo, el robot está muy cerca -> retroceder
    tst   r21
    brmi  control_retroceso

    ; De lo contrario (error 0 o 1), detener el motor
    rjmp  motor_detener

control_avance:
    ; Avanzar: error >= 2
    ; Calcular PWM = 150 * error
    ldi   r23, 150
    mul   r21, r23     ; Multiplicar error x 150 => r1:r0 (16 bits)
    ; Nota: la fórmula real es u = -150*error, pero tomamos la magnitud para PWM
    ; Configurar dirección en PORTB nibble bajo: 1010
    in    temp2, PORTB
    andi  temp2, 0xF0
    ori   temp2, 0x0A
    out   PORTB, temp2
    ; Establecer ciclo PWM con valor calculado
    mov   temp, r1
    sts   OCR1AH, temp
    mov   temp, r0
    sts   OCR1AL, temp
    sts   OCR1BH, r1
    sts   OCR1BL, r0
    rjmp  motor_listo

control_retroceso:
    ; Retroceso: error < 0
    ; Tomar valor absoluto del error
    mov   r22, r21
    neg   r22         ; r22 = |error|
    cpi   r22, 2
    brlo  motor_detener
    ; Calcular PWM = 150 * |error|
    ldi   r23, 150
    mul   r22, r23
    ; Configurar dirección para retroceso: 0101
    in    temp2, PORTB
    andi  temp2, 0xF0
    ori   temp2, 0x05
    out   PORTB, temp2
    ; Ajustar ciclo PWM con valor calculado
    mov   temp, r1
    sts   OCR1AH, temp
    mov   temp, r0
    sts   OCR1AL, temp
    sts   OCR1BH, r1
    sts   OCR1BL, r0
    rjmp  motor_listo

motor_detener:
    ; Detener motor: limpiar nibble bajo de PORTB y poner PWM=0
    in    temp2, PORTB
    andi  temp2, 0xF0
    out   PORTB, temp2
    ldi   temp, 0
    sts   OCR1AH, temp
    sts   OCR1AL, temp
    sts   OCR1BH, temp
    sts   OCR1BL, temp

motor_listo:
    ;------------------------------------------------
    ; Retardo ~100 ms antes de la siguiente medición
    ldi temp2, 255
    rcall retardo_ms
    rjmp bucle_principal

;-----------------------------------------------------------------------------
; Subrutina: retardo_us (aprox. microsegundos)
; Entrada: temp2 contiene el conteo de retardo
;-----------------------------------------------------------------------------
retardo_us:
bucle_retardo_us:
    nop
    dec temp2
    brne bucle_retardo_us
    ret

;-----------------------------------------------------------------------------
; Subrutina: retardo_ms (aprox. milisegundos)
; Entrada: temp2 contiene el conteo de retardo
;-----------------------------------------------------------------------------
retardo_ms:
bucle_retardo_ms:
    ldi temp, 250
bucle_interno_ms:
    nop
    dec temp
    brne bucle_interno_ms
    dec temp2
    brne bucle_retardo_ms
    ret

;-----------------------------------------------------------------------------
; Subrutina: calcular_distancia
; Calcula la distancia en centímetros:
;   distancia (cm) = (2 * (contador Timer4)) / 29
; Primero duplicamos la lectura de 16 bits (durationH:durationL),
; luego se mueve a r25:r24 para usar sbiw y realizar la división
; restando 29 repetidamente. El cociente final (nº de restas)
; se devuelve en r16.
;-----------------------------------------------------------------------------
calcular_distancia:
    ; Pasar valor de Timer4 (16 bits) a r25:r24
    mov r24, durationL
    mov r25, durationH

    ; Duplicar el valor de 16 bits => r25:r24
    lsl r24
    rol r25

    clr r16   ; r16 acumula el cociente

bucle_div:
    ; Ver si r25:r24 < 29
    cpi r25, 0
    breq revisar_bajo
    ; De lo contrario, r25:r24 >= 29 => restar 29 usando sbiw
    sbiw r24, 29
    inc r16
    rjmp bucle_div

revisar_bajo:
    ; Byte alto=0, revisar r24
    cpi r24, 29
    brlo fin_div
    sbiw r24, 29
    inc r16
    rjmp bucle_div

fin_div:
    ret

;-----------------------------------------------------------------------------
; Subrutina: transmitir_byte_serial
; Transmitir el byte en r16 por UART
;-----------------------------------------------------------------------------
transmitir_byte_serial:
    lds temp2, UCSR0A
    sbrs temp2, UDRE0
    rjmp transmitir_byte_serial
    sts UDR0, r16
    ret

;-----------------------------------------------------------------------------
; Subrutina: enviar_distancia
; Convierte r16 (valor de distancia) a tres dígitos ASCII y los envía,
; seguido de regreso y salto de línea.
;-----------------------------------------------------------------------------
enviar_distancia:
    mov r24, r16      ; r24 = distancia
    ldi r25, 0        ; r25 = dígito de centenas

    ; Calcular dígito de centenas
    cpi r24, 200
    brlo revisar_100
    ldi r25, 2
    subi r24, 200
    rjmp calcular_decenas

revisar_100:
    cpi r24, 100
    brlo calcular_decenas
    ldi r25, 1
    subi r24, 100

calcular_decenas:
    ; Calcular dígito de decenas
    ldi r26, 0

bucle_decenas:
    cpi r24, 10
    brlo decenas_listo
    subi r24, 10
    inc r26
    rjmp bucle_decenas

decenas_listo:
    ; r24 contiene el dígito de unidades

    ; Transmitir dígito de centenas
    ldi temp, '0'
    add r25, temp
    mov r16, r25
    rcall transmitir_byte_serial

    ; Transmitir dígito de decenas
    ldi temp, '0'
    add r26, temp
    mov r16, r26
    rcall transmitir_byte_serial

    ; Transmitir dígito de unidades
    ldi temp, '0'
    add r24, temp
    mov r16, r24
    rcall transmitir_byte_serial

    ; Enviar regreso y nueva línea
    ldi r16, 0x0D
    rcall transmitir_byte_serial
    ldi r16, 0x0A
    rcall transmitir_byte_serial
    ret
