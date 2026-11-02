#include "Encoder.h"

 Encoder::Encoder(){
    _unit = NULL;
    _pinA =_pinB = GPIO_NUM_NC;
    _rpmFiltered = _rpmRaw = 0.0f;
    _cpr = 0.0f;


}
Encoder::~Encoder(){
    if(_unit != NULL){
        pcnt_unit_stop(_unit);
        pcnt_unit_disable(_unit);
        if(_chA != NULL){
            pcnt_del_channel(_chA);
            _chA = NULL;
        }
        if(_chB != NULL){
            pcnt_del_channel(_chB);
            _chB  = NULL;
        }
        pcnt_del_unit(_unit);
        _unit = NULL;
    }
}

bool Encoder::setup(gpio_num_t pinA, gpio_num_t pinB,  float gearRatio, int PPR, int mode, float Ts){
    if (_unit != NULL) return false;
    _pinA = pinA; _pinB = pinB; _gearRatio = gearRatio; _PPR = PPR; _mode = mode; _Ts = Ts;
    _cpr = mode*gearRatio*PPR;
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -30000,
        .high_limit = 30000,
        .flags = {.accum_count = 1},
    }; //DEfininendo la configuracion inicial, empezamos con los limites inferiores y superiores, limitados por el tamano de bits de conteo que tenemos que son 16 llega a 65mil y tantos pero lo limitamos a -30 y 30 mil y activamos la flag para que si cuente cuando se sature y reinicie.
   if (pcnt_new_unit(&unit_cfg, &_unit) != ESP_OK) return false; //Crea la unidad a partir de la config y te da la ficha en _unit
    pcnt_glitch_filter_config_t filt = {.max_glitch_ns = 1000}; //definimos el filtro 
    pcnt_unit_set_glitch_filter(_unit, &filt); // anadimos el filtro a la unidad
    pcnt_chan_config_t ch_cfg = {
        .edge_gpio_num = pinA,
        .level_gpio_num = pinB
    }; //configuramos los pines del canal 1
     pcnt_new_channel(_unit, &ch_cfg, &_chA);
    pcnt_channel_set_edge_action(_chA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(_chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    // aqui ponemos la parte de la logica de conteo del primer canal
    
     if (_mode == 4){
        pcnt_chan_config_t ch_cfgB = {
        .edge_gpio_num = pinB,
        .level_gpio_num = pinA
    }; //condicionamos la creacion del segundo canal a solamente cuando tenemos el modo 4 que es la cuadratura
    pcnt_new_channel(_unit, &ch_cfgB, &_chB);
    pcnt_channel_set_edge_action(_chB,PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(_chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    // logica inversa al canal 1 para que no se cancelen es una doble invertida ya que al estar desfasados por 90 grados se tiene que compensar asi
    // para iniciar
    }
    pcnt_unit_add_watch_point(_unit, -30000);// los dos limites donde tenemos que contar en el flag
    pcnt_unit_add_watch_point(_unit, 30000); 
   
    pcnt_unit_enable(_unit);
    pcnt_unit_clear_count(_unit);
    pcnt_unit_start(_unit);
    _lastTime = esp_timer_get_time();
    _lastCount = 0;
    return true;
}
void Encoder::reset(){
    if(_unit == NULL) return;
    pcnt_unit_get_count(_unit, &_lastCount);
    _lastTime = esp_timer_get_time();
    for(int i = 0; i <_MAX; i++)
        _buffer[i] = 0;
    _index = 0;
    _rpmFiltered = _rpmRaw = 0;
}
void Encoder::update(){
    int count = 0, dcount = 0;
    if(_unit == NULL) return; // para que se inice siempre primer el setup antes de update
    pcnt_unit_get_count(_unit, &count);
    int64_t now = esp_timer_get_time();
    float dt = 0;
    dcount = count - _lastCount;
    dt = (now - _lastTime)/ 1000000.0f;
    if (dt > 0){
        _rpmRaw = (dcount/_cpr)/dt *60.0f;
    
        if(_filterType == FilterType::MA){
            _buffer[_index] = _rpmRaw;
            _index = (_index + 1)% _n;
            float sum = 0;
            for(int i = 0; i < _n ; i++)
                sum += _buffer[i];
            _rpmFiltered = sum / _n;
        }
        else if(_filterType == FilterType::EMA)
            _rpmFiltered = _alpha*_rpmRaw + (1-_alpha) *_rpmFiltered;
        else 
            _rpmFiltered = _rpmRaw;
    }
    _lastCount = count;
    _lastTime = now;
}

float Encoder::getRPM(RPMtype getType){
    if(getType == RPMtype::FILTERED)
        return _rpmFiltered;
    else if(getType == RPMtype::RAW)
        return _rpmRaw;
    else
        return 0;
}

void Encoder::setFilter(float param, FilterType filtertype){
    _filterType = filtertype;
    if (_filterType == FilterType::EMA){
        if(param >  0 && param <= 1)
            _alpha = param;   
    }
    else if(_filterType == FilterType::MA){
        if(param <= _MAX && param >= 1)
            _n = (int)param;
        for(int i = 0; i < _MAX ; i++)
                _buffer[i] = 0;
        _index = 0;
    }
        
}
