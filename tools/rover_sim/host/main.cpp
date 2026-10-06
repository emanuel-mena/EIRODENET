#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "cJSON.h"
#include "app_mode.hpp"
#include "competition_runtime.hpp"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "tinyml_policy.hpp"
#include "vision_contract.hpp"
#include "vision_service.hpp"

static uint64_t clock_ms;
static vision_status_t vision;
static rover_imu_state_t imu;
static rover_sensor_state_t sensors;
static peer_comms_status_t peer;
static peer_assignment_t incoming;
static bool have_assignment, rpc_policy;
static cJSON *outbox;
static int left_motor, right_motor;
extern int competition_sim_phase();

static cJSON *get(const cJSON *j, const char *k) { return cJSON_GetObjectItemCaseSensitive(j,k); }
static double num(const cJSON *j,const char *k,double d=0) { auto x=get(j,k); return cJSON_IsNumber(x)?x->valuedouble:d; }
static bool flag(const cJSON *j,const char *k) { return cJSON_IsTrue(get(j,k)); }
static double at(const cJSON *j,int i) { auto x=cJSON_GetArrayItem(j,i); return x?x->valuedouble:0; }
static void n(cJSON *j,const char *k,double v) { cJSON_AddNumberToObject(j,k,v); }
static int color(const cJSON *j) { const char *s=cJSON_GetStringValue(get(j,"color")); return s&&std::string(s)=="green"?0:s&&std::string(s)=="blue"?1:2; }

int64_t esp_timer_get_time() { return clock_ms*1000; }
app_mode_t app_mode_get() { return APP_MODE_COMPETITION; }
uint32_t app_mode_generation() { return 1; }
void vision_service_get_status(vision_status_t *s) {
    *s=vision;
    const uint32_t elapsed = vision.received_ms ? clock_ms-vision.received_ms : 0;
    s->age_ms += elapsed;s->peer_age_ms += elapsed;
    s->pose_valid = s->pose_valid && s->age_ms<=750;
    s->peer_valid = s->peer_valid && s->peer_age_ms<=750;
    for(auto &o:s->obstacles)o.age_ms+=elapsed;
    for(int i=0;i<3;i++){s->cubes[i].age_ms+=elapsed;s->cube_valid[i]=s->cube_valid[i]&&s->cubes[i].age_ms<=750;}
}
void rover_service_get_imu(rover_imu_state_t *s) { *s=imu; }
void rover_service_get_sensors(rover_sensor_state_t *s) { *s=sensors; }
void peer_comms_service_get_status(peer_comms_status_t *s) { *s=peer; }
bool peer_comms_service_get_assignment(peer_assignment_t *s) { *s=incoming; return have_assignment; }
esp_err_t motor_adapter_set(int16_t l,int16_t r) {
    auto pwm=[](int v){return v? (v<0?-1:1)*std::clamp(std::abs(v),700,1000):0;};
    left_motor=pwm(l);right_motor=pwm(r);return ESP_OK;
}
esp_err_t motor_adapter_stop() { return motor_adapter_set(0,0); }
esp_err_t peer_comms_service_send_assignment(const peer_assignment_t *assignment) {
    auto j=cJSON_CreateObject();n(j,"id",assignment->id);n(j,"color",assignment->color);
    cJSON_AddItemToArray(outbox,j);return ESP_OK;
}

extern "C" bool tinyml_host_infer(const float observation[TINYML_POLICY_INPUTS],
                                  float output[TINYML_POLICY_OUTPUTS]) {
    if (!rpc_policy) return false;
    auto request=cJSON_CreateObject();cJSON_AddStringToObject(request,"type","inference");
    auto values=cJSON_AddArrayToObject(request,"observation");
    for(size_t i=0;i<TINYML_POLICY_INPUTS;i++)cJSON_AddItemToArray(values,cJSON_CreateNumber(observation[i]));
    char *serialized=cJSON_PrintUnformatted(request);std::cout<<serialized<<std::endl;
    cJSON_free(serialized);cJSON_Delete(request);
    std::string line;if(!std::getline(std::cin,line))return false;
    auto response=cJSON_Parse(line.c_str());if(!response)return false;
    auto action=get(response,"action");const bool valid=cJSON_IsArray(action)&&cJSON_GetArraySize(action)==2;
    if(valid){output[0]=at(action,0);output[1]=at(action,1);}cJSON_Delete(response);return valid;
}

static bool read_frame(cJSON *f,int id) {
    const cJSON *own=nullptr;uint16_t cols=0,rows=0;float cell=0;
    if(!vision_contract_validate(f,&own,id,&cols,&rows,&cell)) return false;
    vision={};vision.configured=vision.connected=vision.protocol_valid=true;
    vision.sequence=num(f,"seq");vision.frame_timestamp_ms=num(f,"ts_ms");
    vision.received_ms=vision.last_valid_frame_ms=clock_ms;
    const char *phase_value=cJSON_GetStringValue(get(f,"phase"));
    const std::string phase=phase_value?phase_value:"";
    vision.phase=phase=="READY"?VISION_PHASE_READY:phase=="RUNNING"?VISION_PHASE_RUNNING:phase=="FINISHED"?VISION_PHASE_FINISHED:VISION_PHASE_IDLE;
    auto grid=get(f,"grid");vision.grid_cols=num(grid,"cols");vision.grid_rows=num(grid,"rows");vision.cell_mm=num(grid,"cell_mm");
    vision.cube_side=num(f,"cube_side");vision.depot_length=num(get(f,"depot_size"),"length");vision.depot_depth=num(get(f,"depot_size"),"depth");
    cJSON *p;
    cJSON_ArrayForEach(p,get(f,"rovers")) {
        if(num(p,"id")==id) { vision.pose_valid=true;vision.rover_id=id;vision.col=num(p,"col");vision.row=num(p,"row");vision.theta_deg=num(p,"theta");vision.age_ms=num(p,"age_ms"); }
        else {vision.peer_valid=true;vision.peer_id=num(p,"id");vision.peer_col=num(p,"col");vision.peer_row=num(p,"row");vision.peer_theta_deg=num(p,"theta");vision.peer_age_ms=num(p,"age_ms");}
    }
    cJSON_ArrayForEach(p,get(f,"cubes")) {int c=color(p);vision.cube_valid[c]=true;vision.cube_in_depot[c]=flag(p,"in_depot");vision.cubes[c]={(float)num(p,"col"),(float)num(p,"row"),(uint32_t)num(p,"age_ms")};}
    cJSON_ArrayForEach(p,get(f,"depots")) {int c=color(p);vision.depot_valid[c]=true;vision.depot_col[c]=num(p,"col");vision.depot_row[c]=num(p,"row");}
    cJSON_ArrayForEach(p,get(f,"obstacles")) {if(vision.obstacle_count<32)vision.obstacles[vision.obstacle_count++]={(float)num(p,"col"),(float)num(p,"row"),(uint32_t)num(p,"age_ms")};}
    return true;
}

int main(int argc,char **argv) {
    const int id=argc>1?std::stoi(argv[1]):10;rpc_policy=argc>2&&std::string(argv[2])=="--policy-rpc";
    navigation_service_start();navigation_service_set_heading_calibration(0,true);tinyml_policy_start();
    tinyml_policy_set_host_model(rpc_policy?1:0,0,rpc_policy?0x54464c33U:0,rpc_policy);
    uint64_t last_peer=0,previous_step=0;std::string line;
    while(std::getline(std::cin,line)) {
        auto j=cJSON_Parse(line.c_str());if(!j)return 2;
        auto step=(uint64_t)num(j,"step");if(step!=previous_step+1){cJSON_Delete(j);return 3;}previous_step=step;
        clock_ms=num(j,"time_ms");outbox=cJSON_CreateArray();
        if(cJSON_IsObject(get(j,"vision"))&&!read_frame(get(j,"vision"),id)){cJSON_Delete(outbox);cJSON_Delete(j);return 4;}
        auto s=get(j,"sensors");imu.timestamp_ms=clock_ms;imu.valid=imu.calibration_valid=true;
        imu.sample.accel_g[2]=1;imu.sample.gyro_dps[2]=num(s,"gyro");
        sensors.timestamp_ms=num(s,"timestamp_ms",clock_ms);sensors.infrared_valid=true;sensors.ultrasonic_valid=flag(s,"ultrasonic_valid");
        sensors.ultrasonic_error=num(s,"ultrasonic_error",sensors.ultrasonic_valid?ESP_OK:ESP_ERR_TIMEOUT);sensors.distance_mm=num(s,"distance_mm");
        auto ir=get(s,"ir");sensors.infrared={(uint16_t)at(ir,0),(uint16_t)at(ir,1),(uint16_t)at(ir,2),(uint16_t)at(ir,3)};
        auto p=get(j,"peer");if(cJSON_IsObject(p)){
            last_peer=clock_ms;peer.connected=true;peer.mode=APP_MODE_COMPETITION;
            peer.competition_available=flag(p,"available");peer.competition_delivered_mask=num(p,"delivered");
            peer.assignment_id=num(p,"assignment_id");peer.assignment_color=num(p,"assignment_color");
            peer.assignment_result=num(p,"assignment_result");peer.assignment_phase=num(p,"phase");
            peer.model_available=flag(p,"model_available");
            peer.model_version=num(p,"model_version",1);peer.model_crc32=num(p,"model_crc32");
            peer.vision_heading_calibrated=true;auto ack=get(p,"ack");
            if(ack){peer.assignment_ack_id=num(ack,"id");peer.assignment_ack_accepted=true;}
        }
        peer.connected=last_peer&&clock_ms-last_peer<=1500;
        auto ack=cJSON_CreateObject();cJSON *message;
        cJSON_ArrayForEach(message,get(j,"messages")){
            const uint32_t assignment_id=num(message,"id");const uint8_t assignment_color=num(message,"color");
            if(assignment_id&&assignment_color<3){
                if(!have_assignment||incoming.id!=assignment_id)incoming={assignment_id,assignment_color,0};
                have_assignment=true;cJSON_DeleteItemFromObject(ack,"id");n(ack,"id",assignment_id);
            }
        }
        navigation_service_tick();navigation_status_t before;navigation_service_get_status(&before);
        competition_runtime_tick(id==10?COMPETITION_ROLE_COMMANDER:COMPETITION_ROLE_SOLDIER,1);
        navigation_status_t ns;navigation_service_get_status(&ns);competition_assignment_status_t assignment={};
        competition_runtime_get_assignment_status(&assignment);tinyml_policy_status_t model={};tinyml_policy_get_status(&model);
        auto o=cJSON_CreateObject();cJSON_AddStringToObject(o,"type","step");n(o,"step",step);n(o,"left",left_motor);n(o,"right",right_motor);
        n(o,"phase",competition_sim_phase());n(o,"navigation_phase",ns.phase);n(o,"failure",assignment.failure_reason?assignment.failure_reason:(ns.failure_reason?ns.failure_reason:before.failure_reason));
        n(o,"delivered",competition_runtime_delivered_mask());n(o,"assignment_id",assignment.id);n(o,"assignment_color",assignment.color);
        n(o,"assignment_result",assignment.result);n(o,"yields",assignment.yield_count);n(o,"reassignments",assignment.reassignment_count);
        n(o,"blocked_mask",assignment.blocked_mask);
        cJSON_AddBoolToObject(o,"model_available",model.available);
        n(o,"model_version",model.version);n(o,"model_crc32",model.crc32);
        cJSON_AddBoolToObject(o,"available",competition_runtime_available());cJSON_AddItemToObject(o,"messages",outbox);cJSON_AddItemToObject(o,"ack",ack);
        cJSON_AddArrayToObject(o,"route");char *serialized=cJSON_PrintUnformatted(o);std::cout<<serialized<<std::endl;
        cJSON_free(serialized);cJSON_Delete(o);cJSON_Delete(j);
    }
}
