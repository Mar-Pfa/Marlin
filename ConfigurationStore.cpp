#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V15"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homeing);
  #ifdef DELTA
    EEPROM_WRITE_VAR(i,delta_radius);
    EEPROM_WRITE_VAR(i,delta_diagonal_rod);
    EEPROM_WRITE_VAR(i,max_pos);
    EEPROM_WRITE_VAR(i,endstop_adj);
    EEPROM_WRITE_VAR(i,tower_adj);
    EEPROM_WRITE_VAR(i,diagrod_adj);
    EEPROM_WRITE_VAR(i,z_probe_offset);
  #endif
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  #ifndef DOGLCD
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifdef EEPROM_CHITCHAT
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHO("M92 X"); SERIAL_ECHO(axis_steps_per_unit[0]);
    SERIAL_ECHO(" Y"); SERIAL_ECHO(axis_steps_per_unit[1]);
    SERIAL_ECHO(" Z"); SERIAL_ECHO(axis_steps_per_unit[2]);
    SERIAL_ECHO(" E"); SERIAL_ECHO(axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");
      
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHO("  M203 X"); SERIAL_ECHO(max_feedrate[0]);
    SERIAL_ECHO(" Y"); SERIAL_ECHO(max_feedrate[1] ); 
    SERIAL_ECHO(" Z"); SERIAL_ECHO( max_feedrate[2] ); 
    SERIAL_ECHO(" E"); SERIAL_ECHO( max_feedrate[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHO("  M201 X"); SERIAL_ECHO(max_acceleration_units_per_sq_second[0] ); 
    SERIAL_ECHO(" Y" ); SERIAL_ECHO(max_acceleration_units_per_sq_second[1] ); 
    SERIAL_ECHO(" Z" ); SERIAL_ECHO(max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHO(" E" ); SERIAL_ECHO(max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHO("  M204 S"); SERIAL_ECHO(acceleration ); 
    SERIAL_ECHO(" T" ); SERIAL_ECHO(retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHOLN("");
    SERIAL_ECHOLN("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHO("  M205 S"); SERIAL_ECHO(minimumfeedrate ); 
    SERIAL_ECHO(" T" ); SERIAL_ECHO(mintravelfeedrate ); 
    SERIAL_ECHO(" B" ); SERIAL_ECHO(minsegmenttime ); 
    SERIAL_ECHO(" X" ); SERIAL_ECHO(max_xy_jerk ); 
    SERIAL_ECHO(" Z" ); SERIAL_ECHO(max_z_jerk);
    SERIAL_ECHO(" E" ); SERIAL_ECHO(max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHO("  M206 X"); SERIAL_ECHO(add_homeing[0] );
    SERIAL_ECHO(" Y" ); SERIAL_ECHO(add_homeing[1] );
    SERIAL_ECHO(" Z" ); SERIAL_ECHO(add_homeing[2] );
    SERIAL_ECHOLN("");
    #ifdef DELTA
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Delta Geometry adjustment:");
      SERIAL_ECHO_START;
      SERIAL_ECHO("  M666 A"); SERIAL_PROTOCOL_F(tower_adj[0],3);
      SERIAL_ECHO(" B");SERIAL_PROTOCOL_F(tower_adj[1],3);
      SERIAL_ECHO(" C");SERIAL_PROTOCOL_F(tower_adj[2],3);
      SERIAL_ECHO(" I");SERIAL_PROTOCOL_F(tower_adj[3],3);
      SERIAL_ECHO(" J");SERIAL_PROTOCOL_F(tower_adj[4],3);
      SERIAL_ECHO(" K");SERIAL_PROTOCOL_F(tower_adj[5],3);
      SERIAL_ECHO(" U");SERIAL_PROTOCOL_F(diagrod_adj[0],3);
      SERIAL_ECHO(" V");SERIAL_PROTOCOL_F(diagrod_adj[1],3);
      SERIAL_ECHO(" W");SERIAL_PROTOCOL_F(diagrod_adj[2],3);
      SERIAL_ECHO(" R"); SERIAL_ECHO(delta_radius);
      SERIAL_ECHO(" D"); SERIAL_ECHO(delta_diagonal_rod);
      SERIAL_ECHO(" H"); SERIAL_ECHO(max_pos[2]);
      SERIAL_ECHOLN("");
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Endstop Offsets:");
      SERIAL_ECHO_START;
      SERIAL_ECHO("  M666 X"); SERIAL_ECHO(endstop_adj[0]);
      SERIAL_ECHO(" Y"); SERIAL_ECHO(endstop_adj[1]);
      SERIAL_ECHO(" Z"); SERIAL_ECHO(endstop_adj[2]);
      SERIAL_ECHOLN("");
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("Z-Probe Offset:");
      SERIAL_ECHO_START;
      SERIAL_ECHO("  M666 P X"); SERIAL_ECHO(z_probe_offset[0]);
      SERIAL_ECHO(" Y"); SERIAL_ECHO(z_probe_offset[1]);
      SERIAL_ECHO(" Z"); SERIAL_ECHO(z_probe_offset[2]);
      SERIAL_ECHOLN("");
    #endif
 #ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHO("   M301 P"); SERIAL_ECHO(Kp); 
    SERIAL_ECHO(" I"); SERIAL_ECHO(unscalePID_i(Ki)); 
    SERIAL_ECHO(" D"); SERIAL_ECHO(unscalePID_d(Kd));
    SERIAL_ECHOLN(""); 
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
        #ifdef DELTA
          EEPROM_READ_VAR(i,delta_radius);
	    EEPROM_READ_VAR(i,delta_diagonal_rod);
          EEPROM_READ_VAR(i,max_pos);
          EEPROM_READ_VAR(i,endstop_adj);
          EEPROM_READ_VAR(i,tower_adj);
          EEPROM_READ_VAR(i,diagrod_adj);
          EEPROM_READ_VAR(i,z_probe_offset);
          // Update delta constants for updated delta_radius & tower_adj values
          set_delta_constants();
        #endif
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);
        #ifndef DOGLCD
        int lcd_contrast;
        #endif
        EEPROM_READ_VAR(i,lcd_contrast);
    
		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    Config_PrintSettings();
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
    #ifdef DELTA
      delta_radius = DEFAULT_DELTA_RADIUS;
      delta_diagonal_rod = DEFAULT_DELTA_DIAGONAL_ROD;
      endstop_adj[0] = TOWER_A_ENDSTOP_ADJ;
      endstop_adj[1] = TOWER_B_ENDSTOP_ADJ;
      endstop_adj[2] = TOWER_C_ENDSTOP_ADJ;
      tower_adj[0] = TOWER_A_POSITION_ADJ;
      tower_adj[1] = TOWER_B_POSITION_ADJ;
      tower_adj[2] = TOWER_C_POSITION_ADJ;
      tower_adj[3] = TOWER_A_RADIUS_ADJ;
      tower_adj[4] = TOWER_B_RADIUS_ADJ;
      tower_adj[5] = TOWER_C_RADIUS_ADJ;
      diagrod_adj[0] = TOWER_A_DIAGROD_ADJ;
      diagrod_adj[1] = TOWER_B_DIAGROD_ADJ;
      diagrod_adj[2] = TOWER_C_DIAGROD_ADJ;
      max_pos[2] = MANUAL_Z_HOME_POS;
      set_default_z_probe_offset();
      set_delta_constants();
    #endif
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

SERIAL_ECHO_START;
SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");

}


