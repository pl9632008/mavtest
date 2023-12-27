#include <common/mavlink.h>
#include<iostream>

struct DroneInfo{
    int msg_id;
    
    //ATTITUDE #30
    uint32_t time_boot_ms; //Timestamp (time since system boot).
    float roll; //Roll angle (-pi..+pi)
    float pitch; //Pitch angle (-pi..+pi)
    float yaw; //Yaw angle (-pi..+pi)
    float rollspeed; //Roll angular speed
    float pitchspeed; //Pitch angular speed
    float yawspeed; //Yaw angular speed

    //ATTITUDE_QUATERNION #31
    // uint32_t time_boot_ms; 
    float q1; //Quaternion component 1, w (1 in null-rotation)
    float q2; //Quaternion component 2, x (0 in null-rotation)
    float q3; //Quaternion component 3, y (0 in null-rotation)
    float q4; //Quaternion component 4, z (0 in null-rotation)
    // float rollspeed; //Roll angular speed
    // float pitchspeed; //Pitch angular speed
    // float yawspeed; //Yaw angular speed

    //GLOBAL_POSITION_INT #33
    int32_t lat; //Latitude, expressed. Units:degE7
    int32_t lon; //Longitude, expressed. Units:degE7
    int32_t alt; //Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL. Units:mm
    // int32_t relative_alt; //Altitude above ground
    // int16_t vx;//Ground X Speed (Latitude, positive north)
    // int16_t vy;//Ground Y Speed (Longitude, positive east)
    // int16_t vz;//Ground Z Speed (Altitude, positive down)
    // uint16_t hdg;//Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX


    //GPS_RAW_INT #24 
    // int32_t lat; //Latitude (WGS84, EGM96 ellipsoid). Units:degE7
    // int32_t lon; //Longitude (WGS84, EGM96 ellipsoid). Units:degE7
    // int32_t alt; //Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude. Units:mm
    // uint16_t eph; //GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    // uint16_t epv; //GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    // uint16_t vel; //GPS ground speed. If unknown, set to: UINT16_MAX
    // uint16_t cog; //Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

};

void mavMessage(char ch, mavlink_message_t & msg, mavlink_status_t & status, DroneInfo & droneinfo){ 

    memset(&droneinfo, 0, sizeof(DroneInfo));

    if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status)){

        // printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
        
        switch (msg.msgid){
            case MAVLINK_MSG_ID_ATTITUDE:
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                droneinfo.msg_id = MAVLINK_MSG_ID_ATTITUDE;
                droneinfo.roll = attitude.roll;
                droneinfo.pitch = attitude.pitch;
                droneinfo.yaw = attitude.yaw;
                break;
            
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                mavlink_attitude_quaternion_t attitude_quaternion;
                mavlink_msg_attitude_quaternion_decode(&msg, &attitude_quaternion);
                droneinfo.msg_id = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
                droneinfo.q1 = attitude_quaternion.q1;
                droneinfo.q2 = attitude_quaternion.q2;
                droneinfo.q3 = attitude_quaternion.q3;
                droneinfo.q4 = attitude_quaternion.q4;
                break;

            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavlink_global_position_int_t global_position_int;
                mavlink_msg_global_position_int_decode(&msg, &global_position_int);
                droneinfo.msg_id = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
                droneinfo.lat = global_position_int.lat;
                droneinfo.lon = global_position_int.lon;
                droneinfo.alt = global_position_int.alt;
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                mavlink_gps_raw_int_t gps_raw_int;
                mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
                droneinfo.msg_id = MAVLINK_MSG_ID_GPS_RAW_INT;
                droneinfo.lat = gps_raw_int.lat;
                droneinfo.lon = gps_raw_int.lon;
                droneinfo.alt = gps_raw_int.alt;
                break;

            default:
                break;
        }

    }
};


int main(){

    char buffer[2048]={};

    buffer[0]=0xFE;
    buffer[1]=0x1C;
    buffer[2]=0x8F;
    buffer[3]=0x01;
    buffer[4]=0x01;
    buffer[5]=0x1E;
    buffer[6]=0x57;
    buffer[7]=0xDC;
    buffer[8]=0x03;
    buffer[9]=0x00;
    buffer[10]=0x90;
    buffer[11]=0xB5;
    buffer[12]=0xA5;
    buffer[13]=0x3D;
    buffer[14]=0x2B;
    buffer[15]=0xAE;
    buffer[16]=0xD4;
    buffer[17]=0x3D;
    buffer[18]=0xF1;
    buffer[19]=0x01;
    buffer[20]=0xC5;
    buffer[21]=0xBF;
    buffer[22]=0x27;
    buffer[23]=0xFB;
    buffer[24]=0xD6;
    buffer[25]=0x3D;
    buffer[26]=0x12;
    buffer[27]=0x8E;
    buffer[28]=0x46;
    buffer[29]=0x3E;
    buffer[30]=0x2C;
    buffer[31]=0x51;
    buffer[32]=0xBB;
    buffer[33]=0xBE;
    buffer[34]=0xA2;
    buffer[35]=0xBC;

    buffer[36]=0xFE;
    buffer[37]=0x16;
    buffer[38]=0x90;
    buffer[39]=0x01;
    buffer[40]=0x01;
    buffer[41]=0x23;
    buffer[42]=0x00;
    buffer[43]=0x00;
    buffer[44]=0x00;
    buffer[45]=0x00;
    buffer[46]=0x00;
    buffer[47]=0x00;
    buffer[48]=0x00;
    buffer[49]=0x00;
    buffer[50]=0x00;
    buffer[51]=0x00;
    buffer[52]=0x00;
    buffer[53]=0x00;
    buffer[54]=0x00;
    buffer[55]=0x00;
    buffer[56]=0x00;
    buffer[57]=0x00;
    buffer[58]=0x00;
    buffer[59]=0x00;
    buffer[60]=0x00;
    buffer[61]=0x00;
    buffer[62]=0x00;
    buffer[63]=0x00;
    buffer[64]=0x75;
    buffer[65]=0x03;

    buffer[66]=0xFE;
    buffer[67]=0x1E;
    buffer[68]=0x97;
    buffer[69]=0x01;
    buffer[70]=0x01;
    buffer[71]=0x18;
    buffer[72]=0xCB;
    buffer[73]=0xDD;
    buffer[74]=0x03;
    buffer[75]=0x00;
    buffer[76]=0x00;
    buffer[77]=0x00;
    buffer[78]=0x00;
    buffer[79]=0x00;
    buffer[80]=0x00;
    buffer[81]=0x00;
    buffer[82]=0x00;
    buffer[83]=0x00;
    buffer[84]=0x00;
    buffer[85]=0x00;
    buffer[86]=0x00;
    buffer[87]=0x00;
    buffer[88]=0x00;
    buffer[89]=0x00;
    buffer[90]=0x00;
    buffer[91]=0x00;
    buffer[92]=0x63;
    buffer[93]=0x00;
    buffer[94]=0x63;
    buffer[95]=0x00;
    buffer[96]=0x00;
    buffer[97]=0x00;
    buffer[98]=0xFF;
    buffer[99]=0xFF;
    buffer[100]=0x01;
    buffer[101]=0x00;
    buffer[102]=0x7C;
    buffer[103]=0xF3;


    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;
    DroneInfo droneinfo;

    for(int i = 0; i < 2048; i++){

        mavMessage(buffer[i],msg,status,droneinfo);

        if(droneinfo.msg_id == MAVLINK_MSG_ID_ATTITUDE){
            printf("droneinfo.msg_id = %d, roll = %f, pitch = %f, yaw = %f\n", droneinfo.msg_id, droneinfo.roll, droneinfo.pitch, droneinfo.yaw);
        }
        else if(droneinfo.msg_id == MAVLINK_MSG_ID_ATTITUDE_QUATERNION){
            printf("droneinfo.msg_id = %d, q1 = %f, q2 = %f, q3 = %f, q4 = %f\n", droneinfo.msg_id, droneinfo.q1, droneinfo.q2, droneinfo.q3, droneinfo.q4);
        }
        else if(droneinfo.msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
            printf("droneinfo.msg_id = %d, lat = %d, lon = %d, alt = %d\n", droneinfo.msg_id, droneinfo.lat, droneinfo.lon, droneinfo.alt);
        }
        else if(droneinfo.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT){
            printf("droneinfo.msg_id = %d, lat = %d, lon = %d, alt = %d\n", droneinfo.msg_id, droneinfo.lat, droneinfo.lon, droneinfo.alt);
        }

    }


}