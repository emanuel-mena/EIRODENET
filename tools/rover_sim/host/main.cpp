#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "cJSON.h"
#include "app_mode.hpp"
#include "app_storage.hpp"
#include "competition_runtime.hpp"
#include "competition_service.hpp"
#include "motor_adapter.hpp"
#include "motion_control.hpp"
#include "navigation_geometry.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_contract.hpp"
#include "vision_service.hpp"

static uint64_t clock_ms;
static int rover_id;
static vision_status_t vision;
static rover_imu_state_t imu;
static rover_sensor_state_t sensors;
static peer_comms_status_t peer;
static peer_mission_t incoming;
static peer_delivery_event_t delivery;
static bool have_mission;
static cJSON *outbox;
static int left_motor, right_motor;
static float nav_col, nav_row, nav_heading, nav_speed;
static uint64_t nav_last_ms, nav_frame_ms;
static bool nav_initialized;

static cJSON *get(const cJSON *j, const char *k) { return cJSON_GetObjectItemCaseSensitive(j,k); }
static double num(const cJSON *j,const char *k,double d=0) { auto x=get(j,k); return cJSON_IsNumber(x)?x->valuedouble:d; }
static bool flag(const cJSON *j,const char *k) { return cJSON_IsTrue(get(j,k)); }
static double at(const cJSON *j,int i) { auto x=cJSON_GetArrayItem(j,i); return x?x->valuedouble:0; }
static void n(cJSON *j,const char *k,double v) { cJSON_AddNumberToObject(j,k,v); }
static int color(const cJSON *j) { const char *s=cJSON_GetStringValue(get(j,"color")); return s&&std::string(s)=="green"?0:s&&std::string(s)=="blue"?1:2; }

int64_t esp_timer_get_time() { return clock_ms*1000; }
app_mode_t app_mode_get() { return APP_MODE_COMPETITION; }
uint32_t app_mode_generation() { return 1; }
esp_err_t app_storage_get_config(app_storage_config_t *config) { *config={};config->who_am_i=rover_id;return ESP_OK; }
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
bool peer_comms_service_get_mission(peer_mission_t *s) { if(!have_mission)return false;*s=incoming;return true; }
bool peer_comms_service_get_delivery_event(peer_delivery_event_t *s) { *s=delivery;return delivery.valid; }
void competition_service_get_status(competition_status_t *s) { *s={};s->ready=true;s->motor_trim_pwm=0; }
void navigation_service_get_status(navigation_status_t *s) {
    *s={};
    s->pose_valid=nav_initialized;
    s->pose_col=nav_col;s->pose_row=nav_row;s->theta_deg=nav_heading;
    s->linear_speed_cells_s=nav_speed;s->uncertainty_cells=0.3f;
}
esp_err_t navigation_service_cancel(navigation_cancel_reason_t) { return ESP_OK; }
esp_err_t motor_adapter_set(int16_t l,int16_t r) {
    auto pwm=[](int v){return v? (v<0?-1:1)*std::clamp(std::abs(v),700,1000):0;};
    left_motor=pwm(l);right_motor=pwm(r);return ESP_OK;
}
esp_err_t motor_adapter_stop() { return motor_adapter_set(0,0); }
esp_err_t peer_comms_service_send_mission_fragment(const peer_mission_t *mission,uint8_t fragment) {
    auto j=cJSON_CreateObject();cJSON_AddStringToObject(j,"type","mission");
    n(j,"id",mission->id);n(j,"color",mission->color);n(j,"fragment",fragment);
    auto points=cJSON_AddArrayToObject(j,"points");
    const unsigned start=fragment*PEER_MISSION_FRAGMENT_POINTS;
    for(unsigned i=start;i<mission->point_count&&i<start+PEER_MISSION_FRAGMENT_POINTS;i++) {
        auto point=cJSON_CreateArray();cJSON_AddItemToArray(point,cJSON_CreateNumber(mission->points[i].col));
        cJSON_AddItemToArray(point,cJSON_CreateNumber(mission->points[i].row));cJSON_AddItemToArray(points,point);
    }
    cJSON_AddItemToArray(outbox,j);return ESP_OK;
}
esp_err_t peer_comms_service_send_delivery(uint32_t id,uint8_t color_id) {
    auto j=cJSON_CreateObject();cJSON_AddStringToObject(j,"type","delivery");
    n(j,"id",id);n(j,"color",color_id);n(j,"rover_id",rover_id);
    cJSON_AddItemToArray(outbox,j);return ESP_OK;
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
    rover_id=argc>1?std::stoi(argv[1]):10;
    uint64_t last_peer=0,previous_step=0;std::string line;
    while(std::getline(std::cin,line)) {
        auto j=cJSON_Parse(line.c_str());if(!j)return 2;
        auto step=(uint64_t)num(j,"step");if(step!=previous_step+1){cJSON_Delete(j);return 3;}previous_step=step;
        clock_ms=num(j,"time_ms");outbox=cJSON_CreateArray();
        if(cJSON_IsObject(get(j,"vision"))&&!read_frame(get(j,"vision"),rover_id)){cJSON_Delete(outbox);cJSON_Delete(j);return 4;}
        auto s=get(j,"sensors");imu.timestamp_ms=clock_ms;imu.valid=imu.calibration_valid=true;
        imu.sample.accel_g[2]=1;imu.sample.gyro_dps[2]=num(s,"gyro");
        sensors.timestamp_ms=num(s,"timestamp_ms",clock_ms);sensors.infrared_valid=true;sensors.ultrasonic_valid=flag(s,"ultrasonic_valid");
        sensors.ultrasonic_error=num(s,"ultrasonic_error",sensors.ultrasonic_valid?ESP_OK:ESP_ERR_TIMEOUT);sensors.distance_mm=num(s,"distance_mm");
        auto ir=get(s,"ir");sensors.infrared={(uint16_t)at(ir,0),(uint16_t)at(ir,1),(uint16_t)at(ir,2),(uint16_t)at(ir,3)};
        if(nav_initialized && clock_ms>nav_last_ms) {
            const float dt=std::min(0.1f,(float)(clock_ms-nav_last_ms)/1000.0f);
            const float target=(left_motor+right_motor)*9.0f/2000.0f;
            nav_speed+=(target-nav_speed)*std::min(1.0f,dt/0.10f);
            motion_integrate_axle_pose(&nav_col,&nav_row,&nav_heading,nav_speed,
                                       imu.sample.gyro_dps[2],
                                       navigation_axle_offset_cells(vision.cell_mm),dt);
        }
        nav_last_ms=clock_ms;
        if(vision.pose_valid && (!nav_initialized || vision.frame_timestamp_ms!=nav_frame_ms)) {
            nav_col=vision.col;nav_row=vision.row;nav_heading=vision.theta_deg;
            nav_frame_ms=vision.frame_timestamp_ms;nav_initialized=true;
        }
        auto p=get(j,"peer");if(cJSON_IsObject(p)){
            last_peer=clock_ms;peer.connected=true;peer.mode=APP_MODE_COMPETITION;
            peer.competition_available=flag(p,"available");peer.competition_moving=flag(p,"moving");peer.competition_delivered_mask=num(p,"delivered");
            auto ack=get(p,"ack");if(cJSON_IsObject(ack)){
                peer.mission_ack_id=num(ack,"id");peer.mission_ack_fragment=num(ack,"fragment");peer.mission_ack_accepted=flag(ack,"accepted");
            }
        }
        peer.connected=last_peer&&clock_ms-last_peer<=1500;
        auto ack=cJSON_CreateObject();cJSON *message;
        cJSON_ArrayForEach(message,get(j,"messages")){
            const char *type=cJSON_GetStringValue(get(message,"type"));
            if(!type)continue;
            if(std::string(type)=="mission") {
                const uint32_t id=num(message,"id");const uint8_t c=num(message,"color");
                if(!id||c>=3)continue;
                incoming={};incoming.id=id;incoming.color=c;
                auto points=get(message,"points");incoming.point_count=std::min(PEER_MISSION_FRAGMENT_POINTS,(unsigned)cJSON_GetArraySize(points));
                for(unsigned i=0;i<incoming.point_count;i++){
                    auto point=cJSON_GetArrayItem(points,i);incoming.points[i]={(float)at(point,0),(float)at(point,1)};
                }
                have_mission=true;n(ack,"id",id);n(ack,"fragment",num(message,"fragment"));cJSON_AddBoolToObject(ack,"accepted",true);
            } else if(std::string(type)=="delivery") {
                delivery={};delivery.valid=true;delivery.mission_id=num(message,"id");delivery.color=num(message,"color");
                delivery.rover_id=num(message,"rover_id");delivery.timestamp_ms=clock_ms;
            }
        }
        competition_runtime_tick(rover_id==10?COMPETITION_ROLE_COMMANDER:COMPETITION_ROLE_SOLDIER,1);
        competition_runtime_status_t status={};competition_runtime_get_status(&status);
        auto o=cJSON_CreateObject();cJSON_AddStringToObject(o,"type","step");n(o,"step",step);n(o,"left",left_motor);n(o,"right",right_motor);
        n(o,"phase",status.phase);n(o,"target_color",status.target_color);
        n(o,"target_col",status.target_col);n(o,"target_row",status.target_row);
        n(o,"distance_remaining_cells",status.distance_remaining_cells);
        n(o,"detour_heading_deg",status.detour_heading_deg);
        cJSON_AddBoolToObject(o,"detour_active",status.detour_active);
        cJSON_AddBoolToObject(o,"soldier_paused",status.soldier_paused);
        n(o,"delivered",competition_runtime_delivered_mask());
        cJSON_AddBoolToObject(o,"available",competition_runtime_available());
        cJSON_AddBoolToObject(o,"moving",competition_runtime_moving());
        cJSON_AddItemToObject(o,"messages",outbox);cJSON_AddItemToObject(o,"ack",ack);
        char *serialized=cJSON_PrintUnformatted(o);std::cout<<serialized<<std::endl;
        cJSON_free(serialized);cJSON_Delete(o);cJSON_Delete(j);
    }
}
