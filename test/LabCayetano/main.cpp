#include "definitions.h"
#include <math.h>

// ================== Variables ==================
float vel = 0.0f;          // velocidad filtrada [rpm]
float u = 0.0f;            // duty que se aplica al motor [%]
bool  relay = false;       // estado del LED de histéresis (solo se ve en LabVIEW)
bool  link_ok = false;     // true mientras LabVIEW siga mandando mensajes

uint64_t now = 0;
uint64_t last_control = 0;
uint64_t last_telem = 0;
uint64_t last_rx = 0;

const float Ts = CONTROL_TS_US / 1e6f;
const float alpha = 0.3679f;        // filtro de velocidad: exp(-Ts/tau), Ts = 5 ms, tau = 5 ms
const float N_pos = 116.7f;         // filtro de la derivada del PID de posición (N = 1/Tf, del loop shaping)

// ================== Leer una línea ==================
int read_line(SimpleUART &uart, char *line, size_t size, size_t &used)
{
    static bool too_long = false;
    char c[2];

    while (uart.available() > 0 && uart.read(c, sizeof(c)) == 1) {
        if (c[0] == '\r')
            continue;

        if (c[0] == '\n') {
            line[used] = '\0';
            used = 0;
            if (too_long) {
                too_long = false;
                return -1;
            }
            return 1;
        }

        if (used < size - 1)
            line[used++] = c[0];
        else
            too_long = true;
    }
    return 0;
}

// ================== Interpretar el comando ==================
bool handle_line(const char *line)
{
    float a, b, c, d;

    switch (line[0]) {

    case 'G':   // G,lazo,kp,ki,kd
        if (sscanf(line, "G,%f,%f,%f,%f", &a, &b, &c, &d) != 4) return false;
        if (a == 1) { pidSpeed.reset(); return pidSpeed.setGains(b, c, d, 0); }
        if (a == 2) { pidPos.reset();   return pidPos.setGains(b, c, d, N_pos); }   // posición usa filtro N
        return false;

    case 'M':   // M,modo  (0 PWM, 1 velocidad, 2 posición)
        if (sscanf(line, "M,%f", &a) != 1 || a < 0 || a > 2) return false;
        mode = (int)a;
        ref = 0;
        u = 0;
        motor.setDuty(0);
        pidSpeed.reset();
        pidSpeed.setRef(0);
        pidPos.reset();
        pidPos.setRef(0);
        enc.setAngle(0);                 // la posición actual pasa a ser 0°
        return true;

    case 'R':   // R,valor
        if (sscanf(line, "R,%f", &a) != 1) return false;
        if (mode == 0) { ref = a; u = a; motor.setDuty(a); return true; }
        if (mode == 1 && pidSpeed.setRef(a)) { ref = a; return true; }
        if (mode == 2 && pidPos.setRef(a))   { ref = a; return true; }
        return false;

    case 'S':   // S  (paro)
        motor.setStop();
        mode = 0;
        ref = 0;
        u = 0;
        return true;

    case 'Y':   // Y,w_on,w_off  (umbrales del LED)
        if (sscanf(line, "Y,%f,%f", &a, &b) != 2 || a <= b) return false;
        relay_on = a;
        relay_off = b;
        return true;

    default:
        return false;
    }
}

// ================== Lazo de control (cada 5 ms) ==================
void control_step()
{
    // 1) Medir velocidad en rpm (getSpeed da grados/s)
    float vel_raw = enc.getSpeed() / 6.0f;
    if (isnan(vel_raw) || isinf(vel_raw))    // si el encoder dio un dato inválido, se ignora
        vel_raw = vel;

    // 2) Filtro pasa-bajas de primer orden
    vel = alpha * vel + (1.0f - alpha) * vel_raw;

    // 3) Controlador según el modo
    if (mode == 1) {
        u = pidSpeed.calculate(vel);
        motor.setDuty(u);
    }
    if (mode == 2) {
        u = pidPos.calculate(enc.getAngle());
        motor.setDuty(u);
    }
    // en modo 0 (PWM) el duty lo pone directamente el comando R

    // 4) LED con histéresis
    if (fabsf(vel) > relay_on)  relay = true;
    if (fabsf(vel) < relay_off) relay = false;
}

// ================== Mandar datos a LabVIEW (cada telem_ms) ==================
void send_telemetry()
{
    char frame[112];
    float pos = enc.getAngle();
    float meas = (mode == 2) ? pos : vel;
    float err = (mode == 0) ? 0 : ref - meas;
    int estado = link_ok ? 1 : 3;

    int len = snprintf(frame, sizeof(frame), "D,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                       (int)(now / 1000), mode, estado, ref, pos, vel, err, u, relay);
    uart.write(frame, len);
}

// ================== Main ==================
extern "C" void app_main()
{
    esp_task_wdt_deinit();

    uint8_t enc_pins[2] = {EncoderA, EncoderB};
    enc.setup(enc_pins, DEG_PER_EDGE);

    uint8_t motor_pins[2] = {AIN1, AIN2};
    uint8_t motor_ch[2] = {0, 1};
    motor.setup(motor_pins, motor_ch, nullptr, MOTOR_THRESHOLD);
    motor.setDuty(0);

    pidSpeed.setup(controller::PID, discretizationtype::Euler, Ts, -100.0f, 100.0f);
    pidPos.setup(controller::PIDF, discretizationtype::Euler, Ts, -100.0f, 100.0f);    // PD con filtro

    char line[64];
    size_t used = 0;
    char out[16];

    while (true) {
        now = esp_timer_get_time();

        // ---- Comandos de LabVIEW ----
        int r = read_line(uart, line, sizeof(line), used);
        if (r != 0) {
            last_rx = now;
            link_ok = true; 

            if (line[0] != 'H') {                         // H = heartbeat, no se contesta
                bool ok = (r == 1) && handle_line(line);
                int len = snprintf(out, sizeof(out), "%s,%c\n", ok ? "ACK" : "NAK", line[0]);
                uart.write(out, len);
            }
        }

        // ---- Si LabVIEW deja de hablar, se para el motor ----
        if (link_ok && now - last_rx > LINK_TIMEOUT_US) {
            link_ok = false;
            motor.setStop();
            mode = 0;
            ref = 0;
            u = 0;
        }

        // ---- Control cada CONTROL_TS_US ----
        if (now - last_control >= CONTROL_TS_US) {
            last_control = now;
            control_step();
        }

        // ---- Telemetría cada telem_ms ----
        if (now - last_telem >= (uint64_t)telem_ms * 1000) {
            last_telem = now;
            send_telemetry();
        }
    }
}