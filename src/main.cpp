#include "definitions.h"


float u =0;
float corrinteg = 0;
bool is_state_new = true;
uint64_t start_time = 0;
uint64_t now = 0;
float rpm = 0.0f;
float bufferspeed[10] = {0};
float min = 0;
float max = 0;
float meas = 0;
float RefRPM  = 120.0f;
bool airok = false;
bool soapok = false;
state current_state = INACTIVE, last_state = INACTIVE;

state executePUREAIR(){
    if(is_state_new){
        printf("Iniciando maquina de estados\n");
        start_time = now;
        

    }
    vsco.update();
    if(now - start_time >= T1m){
        min = vsco.vscoMin();
        airok = true;
        printf("min measured %0.4f\n", min);

        return INACTIVE;
    }
    return PUREAIR;
};

state executeInactive(){
    if(is_state_new){
        start_time = now;
        vsco.stop();
        printf("Esperando...\n");
    }
    return INACTIVE;
};
state executePURESOAP(){
    if(is_state_new){
        start_time = now;
        
    }
    vsco.update();
    if(now - start_time >= T1m){
        max = vsco.vscoMax();
        soapok = true;
        printf("max: %0.2f\n", max);
        return INACTIVE;
    }
    return PURESOAP;
};

state executeSOAPMEAS(){
    if(is_state_new){
        start_time = now;

    };
    if(soapok && airok)
        vsco.update();
    else{
        printf("Falta alguna medicion Air o Soap");
        return INACTIVE;
    };
        
    if(now - start_time >= T1m){
        meas = vsco.vscoMeas(soapok, airok);
        printf("meas: %0.2f\n", meas);
        return INACTIVE;
    }
    return SOAPMEAS;
};

state executeWASH(){
    if(is_state_new){
        start_time = now;
        printf("Lavando...\n");
    }
    vsco.lavarvsco();
    if(now - start_time >= Tlavado){
        vsco.stop();
        return INACTIVE;
    }
    return WASH;
};
state executeSTIRR(){
    if(is_state_new){
        start_time = now;
        printf("Moviendo Mezcla...\n");
    }
    vsco.revolvervsco();
    if(now - start_time >= Tlavado){
        vsco.stop();
        return INACTIVE;
    }
    return STIRR;
};

int read_line(SimpleUART &uart, char *line, size_t size, size_t &used)
{
    static bool too_long = false;
    char c[2];

    while (uart.available() > 0 && uart.read(c, sizeof(c)) == 1) {   // one byte at a time
        if (c[0] == '\r')
            continue;                        // ignore CR from terminals

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
            too_long = true;                 // drop the rest of the line
    }
    return 0;
}
int handle_line(const char *line, char *out, size_t out_size){
    char cmd[16];
    int len = 0;
    float rpmfake= 0;
    while (*line == ' ')   // skip leading spaces
        line++;
    if(sscanf(line, "%15s", cmd) != 1)
        return 0;
    if (strcmp(cmd, "AIR") == 0) {
        if (current_state != INACTIVE)
        len = snprintf(out, out_size, "ERR BUSY\n");
    
    else {
        current_state = PUREAIR;
        len = snprintf(out, out_size, "OK AIR\n");
    }
    }
    else if (strcmp(cmd, "SOAP") == 0) {
        if (current_state != INACTIVE)
        len = snprintf(out, out_size, "ERR BUSY\n");
    
    else {
        current_state = PURESOAP;
        len = snprintf(out, out_size, "OK SALVO PRO\n");
    }
    }
    else if (strcmp(cmd, "MEAS") == 0) {
    if (current_state != INACTIVE)
        len = snprintf(out, out_size, "ERR BUSY\n");
    else if (airok && soapok) {
        current_state = SOAPMEAS;
        len = snprintf(out, out_size, "OK A VER QP\n");
    }
    else
        len = snprintf(out, out_size, "ERR NOCAL AIR=%d SOAP=%d\n", airok, soapok);

        
    }
    else if (strcmp(cmd, "WASH") == 0) {
        if (current_state != INACTIVE)
            len = snprintf(out, out_size, "ERR BUSY\n");
    
        else {
            current_state = WASH;
            len = snprintf(out, out_size, "OK DUCHANDOME\n");
    }
    }
    else if (strcmp(cmd, "STIRR") == 0) {
            if (current_state != INACTIVE)
            len = snprintf(out, out_size, "ERR BUSY\n");
    
        else {
            current_state = STIRR;
            len = snprintf(out, out_size, "OK MENEANDO\n");
    }
    }
    else if (strcmp(cmd, "STOP") == 0) {
        current_state = INACTIVE;
        len = snprintf(out, out_size, "MOTOR STOPPED\n");
    }
    else if(strcmp(cmd, "RPM") == 0){
        if (current_state != INACTIVE)
            len = snprintf(out, out_size, "ERR BUSY\n");
        else{
            if(sscanf(line, "RPM %f", &rpmfake) == 1){
                if(vsco.setRef(rpmfake)){
                    airok = false;
                    soapok = false;
                    len = snprintf(out, out_size, "Rpm Updated\n");
                }
                else{
                    len = snprintf(out, out_size, "Err Range\n");
                }
            }
            else
                len = snprintf(out, out_size, "ERR ARGS\n");  
        }
          
    }
    else{
        len = snprintf(out, out_size, "venga no existe\n");
    }
    return len;
}

extern "C" void app_main(){
    SimpleUART uart(115200);
    char line[32];
    size_t used = 0;
    char out[64];
    esp_task_wdt_deinit();
    vsco.setup(50, EncoderA, EncoderB, Enable, LPWM, RPWM,
               controller::PI, discretizationtype::Tustin,
               Kp, Ki, 0.0f, 0.0f, RefRPM);
    TickType_t last = xTaskGetTickCount();
    while (true) {
        now = esp_timer_get_time();
        
        switch(current_state) {
            
            case INACTIVE: current_state = executeInactive(); break;
            case PUREAIR: current_state = executePUREAIR(); break;
            case PURESOAP: current_state = executePURESOAP(); break;
            case SOAPMEAS: current_state = executeSOAPMEAS(); break;
            case WASH: current_state = executeWASH(); break;
            case STIRR: current_state = executeSTIRR(); break;
               // waits <= 10 ms
        }
        int len = 0;
        
        int r = read_line(uart, line, sizeof(line), used);

        if (r == 1) {
            printf("%s\n", line);
            len = handle_line(line, out, sizeof(out));
        }
        else if (r == -1) {
            len = snprintf(out, sizeof(out), "ERR TOOLONG\n");
        }

        if (len > 0) {
            uart.write(out, len);
        }

        if (current_state != last_state) {
            last_state = current_state;
            is_state_new = true;
        } else
            is_state_new = false;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(sampletime/1000));
    }
}
